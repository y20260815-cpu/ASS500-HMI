from pathlib import Path

p = Path(__file__).resolve().parents[1] / "main" / "ui" / "as500_ui.cpp"
s = p.read_text(encoding="utf-8")
start = s.index("static const char *s_sensor_setting_names[16]")
end = s.index("};", start)
block = s[start:end]
lines = block.splitlines()
for i, line in enumerate(lines):
    if '"09   ' in line:
        prefix = line.split('"09   ', 1)[0]
        existing = line.split('",', 1)[0] + '",'
        lines[i] = existing + ' "10   음영영역 배율", "", "", "", "", "", ""'
        break
else:
    raise RuntimeError("sensor setting slot 09 not found")
s = s[:start] + "\n".join(lines) + s[end:]
p.write_text(s, encoding="utf-8")
print(p)
