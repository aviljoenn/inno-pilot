import urllib.request, re

html = urllib.request.urlopen("http://192.168.6.12:8888/", timeout=10).read().decode("utf-8", errors="replace")
m = re.search(r"<script>(.*?)</script>", html, re.DOTALL)
js = m.group(1)

i = 0
results = []
in_line_comment = False
in_block_comment = False
in_single_str = False
in_double_str = False
line_no = 1
BS = chr(92)
BT = chr(96)

while i < len(js):
    ch = js[i]
    if ch == '\n':
        line_no += 1
        in_line_comment = False
        i += 1
        continue
    if in_line_comment:
        i += 1
        continue
    if in_block_comment:
        if js[i:i+2] == '*/':
            in_block_comment = False
            i += 2
        else:
            i += 1
        continue
    if in_single_str:
        if ch == BS:
            i += 2
        elif ch == "'":
            in_single_str = False
            i += 1
        else:
            i += 1
        continue
    if in_double_str:
        if ch == BS:
            i += 2
        elif ch == '"':
            in_double_str = False
            i += 1
        else:
            i += 1
        continue
    if js[i:i+2] == '//':
        in_line_comment = True
        i += 2
        continue
    if js[i:i+2] == '/*':
        in_block_comment = True
        i += 2
        continue
    if ch == "'":
        in_single_str = True
        i += 1
        continue
    if ch == '"':
        in_double_str = True
        i += 1
        continue
    if ch == BT:
        results.append((line_no, 'BACKTICK', repr(js[max(0, i-40):i+40])))
        i += 1
        continue
    if ord(ch) > 127:
        results.append((line_no, f'U+{ord(ch):04X}({repr(ch)})', repr(js[max(0, i-30):i+30])))
        i += 1
        continue
    i += 1

print(f"JS size: {len(js)} bytes, {line_no} lines")
print(f"Suspicious tokens outside comments/strings: {len(results)}")
for r in results[:30]:
    print(f"  Line {r[0]}: {r[1]}  ctx={r[2]}")
