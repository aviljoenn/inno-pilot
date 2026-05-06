"""
Playwright debug test: open web-remote settings, toggle Battery Voltage,
click SAVE, and capture both browser console.log output AND Pi journalctl
output so we can see exactly where the SAVE stalls.
"""
import sys
import time
import threading
import io
from playwright.sync_api import sync_playwright

URL = "http://192.168.6.12:8888"
PI_HOST = "192.168.6.12"
PI_USER = "innopilot"
PI_PASS = "innopilot123"

# Timestamped log helper (UTC-relative seconds from test start)
_t0 = time.monotonic()

def tlog(msg: str) -> None:
    elapsed = time.monotonic() - _t0
    line = f"[{elapsed:6.2f}s] {msg}"
    sys.stdout.buffer.write((line + "\n").encode("utf-8"))
    sys.stdout.buffer.flush()


def tail_pi_journal(stop_event: threading.Event, lines_out: list) -> None:
    """SSH to Pi and follow the web-remote journal; append lines to lines_out."""
    try:
        import paramiko
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)
        # -n 0 = no history, -f = follow, grep for DBG lines only
        _, stdout, _ = ssh.exec_command(
            "journalctl -u inno-pilot-web-remote -f -n 0 2>/dev/null",
            get_pty=False
        )
        stdout.channel.setblocking(False)
        while not stop_event.is_set():
            try:
                chunk = stdout.channel.recv(4096)
                if chunk:
                    for line in chunk.decode("utf-8", errors="replace").splitlines():
                        lines_out.append(line)
            except Exception:
                pass
            time.sleep(0.05)
        ssh.close()
    except Exception as exc:
        lines_out.append(f"[SSH ERROR] {exc}")


def run():
    # Start Pi journal tail in background thread
    pi_lines: list = []
    stop_journal = threading.Event()
    journal_thread = threading.Thread(
        target=tail_pi_journal, args=(stop_journal, pi_lines), daemon=True
    )
    journal_thread.start()
    time.sleep(1.0)  # give SSH a moment to connect before we start the test

    browser_console: list = []

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()

        # Capture every browser console message
        def on_console(msg):
            elapsed = time.monotonic() - _t0
            browser_console.append(f"[{elapsed:6.2f}s] BROWSER {msg.type.upper()}: {msg.text}")
        page.on("console", on_console)

        tlog(f"Navigating to {URL}")
        page.goto(URL, timeout=10000, wait_until="domcontentloaded")
        time.sleep(2.0)  # let SSE settle
        tlog("Page loaded")
        page.screenshot(path="test_before_gear.png")
        tlog("Screenshot: test_before_gear.png")

        # Ensure AP is in OFF mode — settings guard (line 1985) blocks otherwise
        tlog("Waiting for gear button…")
        page.wait_for_selector("#gear-btn", timeout=8000)
        gTogglePos = page.evaluate("gTogglePos")
        tlog(f"Current gTogglePos = {repr(gTogglePos)}")
        if gTogglePos != 'off':
            tlog("Switching to OFF mode via the OFF radio button")
            switched = page.evaluate("""() => {
                var btn = document.querySelector('.mode-radio[data-action="off"]');
                if (btn) { btn.click(); return 'clicked off btn'; }
                return 'off btn not found';
            }""")
            tlog(f"Mode switch: {switched}")
            time.sleep(0.5)
            gTogglePos = page.evaluate("gTogglePos")
            tlog(f"gTogglePos after switch = {repr(gTogglePos)}")

        # Open settings
        tlog("Calling openSettings() via JS")
        page.evaluate("openSettings()")
        time.sleep(1.5)

        # Verify panel opened
        panel_classes = page.evaluate("document.getElementById('sov').className")
        tlog(f"#sov classes after openSettings(): {repr(panel_classes)}")
        page.screenshot(path="test_after_opensettings.png")
        tlog("Screenshot: test_after_opensettings.png")
        if 'hidden' in panel_classes:
            tlog("ERROR: settings panel is still hidden after openSettings() — aborting")
            stop_journal.set()
            browser.close()
            return

        status_el = page.locator("#sov-status")
        tlog("Status after open: " + repr(status_el.inner_text()))

        # Wait for GET to resolve
        time.sleep(3.5)
        tlog("Status after GET settle: " + repr(status_el.inner_text()))

        # Toggle Battery Voltage via JS (label → next sibling button)
        toggled = page.evaluate("""() => {
            const labels = Array.from(document.querySelectorAll('span, label, div, td'));
            for (const el of labels) {
                if (el.textContent.trim() === 'Battery Voltage') {
                    let sib = el.nextElementSibling;
                    while (sib) {
                        const btn = sib.tagName === 'BUTTON' ? sib : sib.querySelector('button');
                        if (btn) { btn.click(); return 'clicked: ' + btn.textContent.trim(); }
                        sib = sib.nextElementSibling;
                    }
                    const p = el.parentElement;
                    if (p) {
                        let psib = p.nextElementSibling;
                        while (psib) {
                            const btn = psib.tagName === 'BUTTON' ? psib : psib.querySelector('button');
                            if (btn) { btn.click(); return 'clicked(parent sib): ' + btn.textContent.trim(); }
                            psib = psib.nextElementSibling;
                        }
                    }
                    return 'label found but no button';
                }
            }
            return 'Battery Voltage label not found';
        }""")
        tlog(f"Battery Voltage toggle: {toggled}")

        # Click SAVE and watch
        tlog("=== Clicking SAVE ===")
        save_start = time.monotonic()
        page.click("#sov-save")

        # Poll status and panel visibility for up to 20 s
        last_status = ""
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            try:
                txt = status_el.inner_text()
            except Exception:
                txt = "<element gone>"
            if txt != last_status:
                elapsed = time.monotonic() - save_start
                tlog(f"  Status changed [{elapsed:.1f}s after SAVE]: {repr(txt)}")
                last_status = txt
            try:
                panel_open = page.is_visible("#sov")
            except Exception:
                panel_open = False
            if not panel_open:
                tlog(f"  Panel closed [{time.monotonic()-save_start:.1f}s after SAVE]")
                break
            time.sleep(0.2)
        else:
            tlog("  20s timeout reached — panel still open, status still: " + repr(last_status))

        page.screenshot(path="test_settings_save_final.png")
        tlog("Screenshot saved: test_settings_save_final.png")
        browser.close()

    # Stop journal tail and give it a moment to drain
    time.sleep(0.5)
    stop_journal.set()
    journal_thread.join(timeout=3)

    # Print browser console output
    print("\n=== BROWSER CONSOLE ===")
    for line in browser_console:
        sys.stdout.buffer.write((line + "\n").encode("utf-8"))
    sys.stdout.buffer.flush()

    # Print Pi journal lines (filter to DBG + WARNING lines to keep it short)
    print("\n=== PI JOURNAL (web-remote) ===")
    for line in pi_lines:
        sys.stdout.buffer.write((line + "\n").encode("utf-8"))
    sys.stdout.buffer.flush()


if __name__ == "__main__":
    run()
