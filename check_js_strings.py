"""
Scan the _HTML triple-quoted Python string for JS string literals
that contain embedded real newlines/tabs (produced by unescaped \n, \t in
the Python source). Any such literal will be a JS syntax error.
"""
src = open("C:/gitsrc/inno-pilot/compute_module/glue/inno_web_remote.py",
           encoding="utf-8").read()

# Extract the _HTML string value as Python would evaluate it
# (already processed by Python's string parser — the actual characters)
html_start = src.find('_HTML = """') + len('_HTML = """')
html_end   = src.find('\n""".replace("$$WHEEL_SVG$$"', html_start)
html_val = src[html_start:html_end]  # raw bytes of the HTML, including real \n etc.

# Find the <script>...</script> range
script_open  = html_val.find('<script>') + len('<script>')
script_close = html_val.rfind('</script>')
js = html_val[script_open:script_close]

# Walk through the JS looking for string literals that contain real
# newline (0x0A), carriage return (0x0D), or NUL (0x00).
# Use a simple state machine (same as before).
BS = '\\'
SQ = "'"
DQ = '"'
BT = '`'

i = 0
line_no = 1
bad = []

in_lc = False   # line comment
in_bc = False   # block comment
str_ch = None   # None, SQ, DQ, or BT
tpl_depth = 0   # template literal depth

while i < len(js):
    ch = js[i]
    if ch == '\n':
        if in_lc:
            in_lc = False
        if str_ch in (SQ, DQ) and str_ch is not None:
            bad.append((line_no, 'newline inside JS string', repr(js[max(0,i-40):i+10])))
        line_no += 1
        i += 1
        continue

    if in_lc:
        i += 1
        continue

    if in_bc:
        if js[i:i+2] == '*/':
            in_bc = False
            i += 2
        else:
            i += 1
        continue

    if str_ch is not None:
        if ch == BS:
            i += 2  # skip escape
            continue
        if ch == str_ch:
            str_ch = None
            i += 1
            continue
        if ch == '\r' or ch == '\0':
            bad.append((line_no, f'ctrl char 0x{ord(ch):02X} inside JS string',
                        repr(js[max(0,i-40):i+10])))
        i += 1
        continue

    # Not in any string/comment
    if js[i:i+2] == '//':
        in_lc = True
        i += 2
        continue
    if js[i:i+2] == '/*':
        in_bc = True
        i += 2
        continue
    if ch in (SQ, DQ, BT):
        str_ch = ch
        i += 1
        continue
    i += 1

print(f"Scanned {line_no} JS lines")
print(f"Bad string literals: {len(bad)}")
for ln, reason, ctx in bad:
    print(f"  JS line {ln}: {reason}")
    print(f"    ctx: {ctx}")
