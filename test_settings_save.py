"""
Playwright test: open web-remote settings, toggle Battery Voltage, SAVE,
and report every status-line state change seen during the operation.
"""
import sys
import time
from playwright.sync_api import sync_playwright

URL = "http://192.168.6.12:8888"

def run():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()

        # Capture console messages from the page
        console_msgs = []
        page.on("console", lambda msg: console_msgs.append(msg.text))

        sys.stdout.buffer.write(b"=== Navigating to " + URL.encode() + b" ===\n")
        sys.stdout.buffer.flush()
        # SSE keeps the page from ever reaching networkidle; use domcontentloaded + settle delay.
        page.goto(URL, timeout=10000, wait_until="domcontentloaded")
        time.sleep(2.0)  # let SSE initial burst settle

        # ---- Open settings panel ----
        sys.stdout.buffer.write(b"\n--- Clicking gear button ---\n")
        sys.stdout.buffer.flush()
        page.click("#gear-btn")
        page.wait_for_selector("#sov:not(.hidden)", timeout=4000)
        time.sleep(0.5)

        status_el = page.locator("#sov-status")
        status_text = status_el.inner_text()
        sys.stdout.buffer.write(("Status after open: " + repr(status_text) + "\n").encode("utf-8"))
        sys.stdout.buffer.flush()

        # Wait for the GET response (loading... -> result)
        time.sleep(3.0)
        status_text = status_el.inner_text()
        sys.stdout.buffer.write(("Status after GET settle: " + repr(status_text) + "\n").encode("utf-8"))
        sys.stdout.buffer.flush()

        # ---- Find and toggle the Battery Voltage setting ----
        # The label text is in a <span> or similar; the ON/OFF buttons immediately follow it.
        # Strategy: find the label element by text, then grab the next sibling button group.
        batt_label = page.locator("text=Battery Voltage").first
        if batt_label.count() == 0:
            sys.stdout.buffer.write(b"WARNING: 'Battery Voltage' text not found in settings panel\n")
            sys.stdout.buffer.flush()
        else:
            sys.stdout.buffer.write(b"Found 'Battery Voltage' label\n")
            sys.stdout.buffer.flush()
            # Click the first .sf-bool-btn that follows the Battery Voltage label
            # They render as: <label>Battery Voltage</label> <button>ON</button> <button>OFF</button>
            # Use JS to find and toggle: locate buttons after the Battery Voltage text node
            toggled = page.evaluate("""() => {
                const labels = Array.from(document.querySelectorAll('span, label, div'));
                for (const el of labels) {
                    if (el.textContent.trim() === 'Battery Voltage') {
                        // Walk siblings until we find a button
                        let sib = el.nextElementSibling;
                        while (sib) {
                            const btn = sib.tagName === 'BUTTON' ? sib
                                      : sib.querySelector('button');
                            if (btn) { btn.click(); return btn.textContent.trim(); }
                            sib = sib.nextElementSibling;
                        }
                        // Try parent's next sibling
                        const parentSib = el.parentElement && el.parentElement.nextElementSibling;
                        if (parentSib) {
                            const btn = parentSib.tagName === 'BUTTON' ? parentSib
                                      : parentSib.querySelector('button');
                            if (btn) { btn.click(); return btn.textContent.trim(); }
                        }
                        return 'label found but no button sibling';
                    }
                }
                return 'label not found';
            }""")
            sys.stdout.buffer.write(("Toggle result: " + repr(toggled) + "\n").encode("utf-8"))
            sys.stdout.buffer.flush()

        # ---- Click SAVE ----
        sys.stdout.buffer.write(b"\n--- Clicking SAVE ---\n")
        sys.stdout.buffer.flush()
        save_time = time.monotonic()
        page.click("#sov-save")

        # Poll the status line every 200 ms for up to 15 s
        last_text = ""
        transitions = []
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                txt = status_el.inner_text()
            except Exception:
                txt = "<panel gone>"
            if txt != last_text:
                elapsed = time.monotonic() - save_time
                transitions.append((elapsed, txt))
                last_text = txt
            # Stop once the panel closes (sov hidden)
            try:
                panel_visible = page.is_visible("#sov")
            except Exception:
                panel_visible = False
            if not panel_visible and last_text != "Saving\u2026":
                time.sleep(0.3)  # let final status render
                break
            time.sleep(0.2)

        sys.stdout.buffer.write(b"\n=== Status line transitions after SAVE ===\n")
        for elapsed, txt in transitions:
            sys.stdout.buffer.write(
                ("[{:5.1f}s] {}\n".format(elapsed, repr(txt))).encode("utf-8")
            )
        sys.stdout.buffer.flush()

        # Take a screenshot of the final state (panel may be closed)
        page.screenshot(path="test_settings_save_final.png")
        sys.stdout.buffer.write(b"\nScreenshot saved: test_settings_save_final.png\n")

        browser.close()

if __name__ == "__main__":
    run()
