import urllib.request, re

raw = urllib.request.urlopen("http://192.168.6.12:8888/", timeout=10).read()
start = raw.find(b"<script>") + 8
end   = raw.rfind(b"</script>")
js_bytes = raw[start:end]

# Check for control characters (0x00-0x1F except tab/LF/CR)
bad_ctrl = []
for i, b in enumerate(js_bytes):
    if b < 0x20 and b not in (0x09, 0x0A, 0x0D):
        line_no = js_bytes[:i].count(0x0A) + 1
        bad_ctrl.append((line_no, i, b))

print(f"Control chars (not tab/LF/CR): {len(bad_ctrl)}")
for line_no, pos, b in bad_ctrl[:20]:
    ctx = js_bytes[max(0, pos-30):pos+30]
    print(f"  line {line_no} pos {pos}: byte 0x{b:02X}  ctx={ctx!r}")

nuls = js_bytes.count(0)
print(f"NUL bytes in JS: {nuls}")

js_str = js_bytes.decode("utf-8", errors="replace")

# Malformed \u escapes: \u not followed by exactly 4 hex digits or {
pat = r'\\u(?![0-9a-fA-F]{4}|[{])'
bad_esc = [(m.start(), m.group(0), js_str[max(0,m.start()-20):m.start()+20])
           for m in re.finditer(pat, js_str)]
print(f"\nPossibly malformed \\\\u escapes: {len(bad_esc)}")
for pos, match, ctx in bad_esc[:10]:
    line_no = js_str[:pos].count('\n') + 1
    print(f"  line {line_no}: match={match!r} ctx={ctx!r}")

# Look for backticks
bt = chr(96)
bt_positions = [i for i, c in enumerate(js_str) if c == bt]
print(f"\nBackticks in full JS: {len(bt_positions)}")
for pos in bt_positions[:5]:
    line_no = js_str[:pos].count('\n') + 1
    print(f"  line {line_no}: ctx={js_str[max(0,pos-30):pos+30]!r}")
