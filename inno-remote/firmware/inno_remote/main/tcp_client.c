#include "tcp_client.h"

#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "wifi_secrets.h"   // BRIDGE_IP
#include "wifi_status.h"    // wifi_ui_get_status()

static const char *TAG = "TCP";

#define BRIDGE_PORT         8555
#define PING_INTERVAL_MS    2000      // bridge timeout is 6 s → 3 chances
#define RECONNECT_DELAY_MS  1000
#define RX_BUF_SIZE         256
#define RECV_TIMEOUT_SEC    4         // detect dead bridge faster than 6 s

static volatile bool s_connected = false;
static int s_sock = -1;
static SemaphoreHandle_t s_sock_mutex = NULL;
static tcp_rx_callback_t      s_rx_cb      = NULL;
static tcp_connect_callback_t s_connect_cb = NULL;

// ---- public API -----------------------------------------------------------

bool tcp_client_is_connected(void)
{
    return s_connected;
}

void tcp_client_set_rx_callback(tcp_rx_callback_t cb)
{
    s_rx_cb = cb;
}

void tcp_client_set_connect_callback(tcp_connect_callback_t cb)
{
    s_connect_cb = cb;
}

bool tcp_client_send(const char *line)
{
    if (!s_connected || s_sock < 0) return false;

    // Build "line\n" in a stack buffer
    char buf[RX_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf), "%s\n", line);
    if (len <= 0 || len >= (int)sizeof(buf)) return false;

    bool ok = false;
    if (xSemaphoreTake(s_sock_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_sock >= 0) {
            int sent = send(s_sock, buf, len, 0);
            ok = (sent == len);
        }
        xSemaphoreGive(s_sock_mutex);
    }
    return ok;
}

// ---- internal -------------------------------------------------------------

static void close_socket(void)
{
    if (xSemaphoreTake(s_sock_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (s_sock >= 0) {
            close(s_sock);
            s_sock = -1;
        }
        s_connected = false;
        xSemaphoreGive(s_sock_mutex);
    }
}

static bool try_connect(void)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket(): %d", errno);
        return false;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(BRIDGE_PORT),
    };
    inet_aton(BRIDGE_IP, &addr.sin_addr);

    // Set receive timeout so recv() doesn't block forever
    struct timeval tv = { .tv_sec = RECV_TIMEOUT_SEC, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "Connecting to %s:%d ...", BRIDGE_IP, BRIDGE_PORT);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGW(TAG, "connect() failed: %d", errno);
        close(sock);
        return false;
    }

    if (xSemaphoreTake(s_sock_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        s_sock = sock;
        s_connected = true;
        xSemaphoreGive(s_sock_mutex);
    }

    ESP_LOGI(TAG, "Connected to bridge");
    if (s_connect_cb) s_connect_cb();
    return true;
}

static void tcp_client_task(void *arg)
{
    char rxbuf[RX_BUF_SIZE];
    int  rxpos = 0;

    while (1) {
        // Wait for Wi-Fi before attempting connection
        while (!wifi_ui_get_status().has_ip) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // Attempt connection
        if (!try_connect()) {
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            continue;
        }

        rxpos = 0;
        uint32_t last_ping = 0;

        // Connected loop: send PINGs, receive lines
        while (s_connected) {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

            // Send PING at interval
            if ((now - last_ping) >= PING_INTERVAL_MS) {
                if (!tcp_client_send("PING")) {
                    ESP_LOGW(TAG, "PING send failed, disconnecting");
                    break;
                }
                last_ping = now;
            }

            // Receive data (with SO_RCVTIMEO, blocks up to RECV_TIMEOUT_SEC)
            int space = (int)sizeof(rxbuf) - rxpos - 1;
            if (space <= 0) {
                // Line too long — discard buffer
                ESP_LOGW(TAG, "RX buffer overflow, discarding");
                rxpos = 0;
                continue;
            }

            int n = recv(s_sock, rxbuf + rxpos, space, 0);
            if (n > 0) {
                rxpos += n;
                rxbuf[rxpos] = '\0';

                // Process complete lines
                char *start = rxbuf;
                char *nl;
                while ((nl = strchr(start, '\n')) != NULL) {
                    *nl = '\0';
                    // Strip trailing \r
                    if (nl > start && *(nl - 1) == '\r') *(nl - 1) = '\0';

                    if (s_rx_cb && *start) {
                        s_rx_cb(start);
                    }

                    start = nl + 1;
                }

                // Move leftover to front of buffer
                int leftover = rxpos - (int)(start - rxbuf);
                if (leftover > 0) {
                    memmove(rxbuf, start, leftover);
                }
                rxpos = leftover;

            } else if (n == 0) {
                // Peer closed connection
                ESP_LOGW(TAG, "Bridge closed connection");
                break;
            } else {
                // n < 0: check if it's just a timeout (expected) or real error
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // recv timeout — not an error, just no data
                    continue;
                }
                ESP_LOGW(TAG, "recv() error: %d", errno);
                break;
            }
        }

        // Disconnected — clean up and retry
        close_socket();
        ESP_LOGI(TAG, "Disconnected, retrying in %d ms", RECONNECT_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
}

void tcp_client_start(void)
{
    s_sock_mutex = xSemaphoreCreateMutex();
    configASSERT(s_sock_mutex);

    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "TCP client task started (bridge %s:%d)", BRIDGE_IP, BRIDGE_PORT);
}
