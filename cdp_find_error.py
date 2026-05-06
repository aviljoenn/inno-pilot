import asyncio
from playwright.async_api import async_playwright

async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        context = await browser.new_context()
        page = await context.new_page()

        # Enable CDP
        cdp = await context.new_cdp_session(page)

        # Collect all CDP runtime exceptions
        exceptions = []
        async def on_exception(params):
            exceptions.append(params)
        cdp.on("Runtime.exceptionThrown", on_exception)

        # Enable runtime events
        await cdp.send("Runtime.enable")
        await cdp.send("Debugger.enable")

        scripts = []
        async def on_script_parsed(params):
            scripts.append(params)
        cdp.on("Debugger.scriptParsed", on_script_parsed)

        script_failed = []
        async def on_script_failed(params):
            script_failed.append(params)
        cdp.on("Debugger.scriptFailedToParse", on_script_failed)

        print("Loading page...")
        await page.goto("http://192.168.6.12:8888/", timeout=15000)
        await asyncio.sleep(2)

        print(f"\n=== Scripts parsed OK: {len(scripts)} ===")
        print(f"\n=== Scripts FAILED to parse: {len(script_failed)} ===")
        for s in script_failed:
            print(f"  url:         {s.get('url', '(inline)')}")
            print(f"  startLine:   {s.get('startLine')}")
            print(f"  errorLine:   {s.get('errorLine')}")
            print(f"  errorMsg:    {s.get('errorMessage')}")

        print(f"\n=== Runtime exceptions: {len(exceptions)} ===")
        for ex in exceptions:
            detail = ex.get("exceptionDetails", {})
            print(f"  text:       {detail.get('text')}")
            print(f"  lineNumber: {detail.get('lineNumber')}")
            print(f"  columnNum:  {detail.get('columnNumber')}")
            ex_obj = detail.get("exception", {})
            print(f"  exception:  {ex_obj.get('description', '')[:200]}")

        await browser.close()

asyncio.run(main())
