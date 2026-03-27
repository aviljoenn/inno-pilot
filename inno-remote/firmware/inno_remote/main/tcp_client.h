#pragma once

#include <stdbool.h>

/**
 * Start the TCP client task.
 * Connects to the bridge, sends PING keepalives, auto-reconnects.
 * Call once from app_main after Wi-Fi is started.
 */
void tcp_client_start(void);

/**
 * Returns true when the TCP connection to the bridge is alive.
 * Thread-safe (atomic read).
 */
bool tcp_client_is_connected(void);

/**
 * Send a line to the bridge (appends \n).
 * Returns true on success, false if not connected or send failed.
 * Thread-safe.
 */
bool tcp_client_send(const char *line);

/**
 * Receive callback type.  Called from the TCP task for each complete
 * line received from the bridge (newline stripped).
 * Register with tcp_client_set_rx_callback() before starting.
 */
typedef void (*tcp_rx_callback_t)(const char *line);

/**
 * Register a callback for received lines.
 * Must be called before tcp_client_start().
 */
void tcp_client_set_rx_callback(tcp_rx_callback_t cb);

/**
 * Callback type invoked immediately after each new TCP connection is
 * established (called from the TCP task, before the first PING is sent).
 * Use it to send a handshake frame on every new connection.
 */
typedef void (*tcp_connect_callback_t)(void);

/**
 * Register a callback that fires once per new bridge connection.
 * Must be called before tcp_client_start().
 */
void tcp_client_set_connect_callback(tcp_connect_callback_t cb);
