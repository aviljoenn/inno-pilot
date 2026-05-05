"""
Playwright test for the OTA software update flow on http://100.125.212.5:8888/
Run with: python test_ota_flow.py
"""

import time
import traceback
from playwright.sync_api import sync_playwright, ConsoleMessage

CONSOLE_ERRORS = []

def log(msg: str) -> None:
    print(msg, flush=True)

def on_console(msg: ConsoleMessage) -> None:
    if msg.type == "error":
        CONSOLE_ERRORS.append(f"[console.error] {msg.text}")

def run_test() -> None:
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        page.on("console", on_console)

        # ── Step 1 ───────────────────────────────────────────────────────────
        log("=== Step 1: Loading page http://100.125.212.5:8888/ ===")
        page.goto("http://100.125.212.5:8888/", timeout=30_000)
        log("  Page loaded. Waiting 8 seconds for bridge connection...")
        time.sleep(8)
        log("  Wait complete.")

        # ── Step 2 ───────────────────────────────────────────────────────────
        log("\n=== Step 2: Setting mode to OFF ===")
        # Inspect DOM for possible OFF button candidates
        candidates = page.eval_on_selector_all(
            "[data-action], input[type=radio], button",
            """els => els.map(el => ({
                tag: el.tagName,
                id: el.id,
                type: el.type || '',
                name: el.name || '',
                value: el.value || '',
                dataAction: el.dataset.action || '',
                text: el.innerText || '',
                class: el.className || ''
            }))"""
        )
        log(f"  Found {len(candidates)} interactive elements. Scanning for OFF...")
        off_btn = None
        for c in candidates:
            combined = (c["text"] + c["id"] + c["value"] + c["dataAction"]).lower()
            log(f"    tag={c['tag']} id={c['id']!r} dataAction={c['dataAction']!r} "
                f"text={c['text'][:40]!r} value={c['value']!r}")
            if "off" in combined:
                log(f"  --> Matched OFF candidate: {c}")
                off_btn = c
                break

        if off_btn:
            selector = None
            if off_btn["id"]:
                selector = f"#{off_btn['id']}"
            elif off_btn["dataAction"]:
                selector = f"[data-action='{off_btn['dataAction']}']"
            elif off_btn["value"]:
                selector = f"[value='{off_btn['value']}']"
            if selector:
                log(f"  Clicking OFF element with selector: {selector}")
                page.click(selector, timeout=5_000)
            else:
                log("  No usable selector found for OFF button; trying to match by text")
                page.get_by_text("OFF", exact=True).first.click()
        else:
            log("  No OFF element found via scan; trying data-action=off fallback")
            try:
                page.click("[data-action='off']", timeout=3_000)
            except Exception:
                log("  data-action=off not found; trying button text 'OFF'")
                try:
                    page.get_by_text("OFF", exact=True).first.click(timeout=3_000)
                except Exception as e:
                    log(f"  WARNING: could not click OFF button: {e}")

        time.sleep(1)
        log("  Mode set to OFF (or best effort). 1 s wait complete.")

        # ── Step 3 ───────────────────────────────────────────────────────────
        log("\n=== Step 3: Clicking gear button (id='gear-btn') ===")
        try:
            page.click("#gear-btn", timeout=5_000)
            log("  gear-btn clicked.")
        except Exception as e:
            log(f"  ERROR clicking gear-btn: {e}")
        time.sleep(2)
        log("  2 s wait complete.")

        # ── Step 4 ───────────────────────────────────────────────────────────
        log("\n=== Step 4: Clicking CHECK > button (id='sf-check-update') ===")
        try:
            page.click("#sf-check-update", timeout=5_000)
            log("  sf-check-update clicked.")
        except Exception as e:
            log(f"  ERROR clicking sf-check-update: {e}")
        log("  Waiting 5 seconds for update check...")
        time.sleep(5)
        log("  Wait complete.")

        # ── Step 5 ───────────────────────────────────────────────────────────
        log("\n=== Step 5: upd-body textContent ===")
        try:
            upd_body_text = page.eval_on_selector(
                "#upd-body", "el => el.textContent"
            )
            log(f"  upd-body.textContent = {upd_body_text!r}")
        except Exception as e:
            log(f"  ERROR reading upd-body: {e}")
            upd_body_text = None

        # ── Step 6 ───────────────────────────────────────────────────────────
        log("\n=== Step 6: upd-overlay className ===")
        try:
            upd_overlay_class = page.eval_on_selector(
                "#upd-overlay", "el => el.className"
            )
            log(f"  upd-overlay.className = {upd_overlay_class!r}")
        except Exception as e:
            log(f"  ERROR reading upd-overlay: {e}")
            upd_overlay_class = None

        # ── Step 7 / 8 ───────────────────────────────────────────────────────
        log("\n=== Step 7: Checking visibility of INSTALL button (id='upd-install') ===")
        install_visible = False
        try:
            install_display = page.eval_on_selector(
                "#upd-install",
                "el => window.getComputedStyle(el).display"
            )
            log(f"  upd-install computed display = {install_display!r}")
            install_visible = install_display != "none"
        except Exception as e:
            log(f"  ERROR checking upd-install: {e}")

        if install_visible:
            log("  INSTALL button is visible. Clicking it...")
            try:
                page.click("#upd-install", timeout=5_000)
                log("  upd-install clicked. Waiting 5 seconds...")
                time.sleep(5)
                try:
                    upd_body_after = page.eval_on_selector(
                        "#upd-body", "el => el.textContent"
                    )
                    log(f"\n=== Step 8: upd-body.textContent after INSTALL click ===")
                    log(f"  {upd_body_after!r}")
                except Exception as e:
                    log(f"  ERROR reading upd-body after INSTALL: {e}")
            except Exception as e:
                log(f"  ERROR clicking upd-install: {e}")
        else:
            log("\n=== Step 8: INSTALL button was NOT shown (display:none or missing) ===")

        # ── Step 9 ───────────────────────────────────────────────────────────
        log("\n=== Step 9: JS console errors collected throughout test ===")
        if CONSOLE_ERRORS:
            for err in CONSOLE_ERRORS:
                log(f"  {err}")
        else:
            log("  (no JS console errors)")

        browser.close()

if __name__ == "__main__":
    try:
        run_test()
    except Exception:
        print("FATAL ERROR in test runner:")
        traceback.print_exc()
