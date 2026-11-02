"""Record local build evidence without opening real serial hardware."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import zipfile

root=Path(__file__).parent.resolve()
desktop=Path("C:/Users/Administrator/Desktop")
original=desktop/"WHEELTEC串口调试助手.exe"
backup=desktop/"小车工具原版备份_20261027/WHEELTEC串口调试助手.exe"
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
baseline=desktop/"jiezhezuo/Robot_Treasure-main/Robot_Treasure-main/Tools/MapTimingLogger"
modules=("laser_live_server.py","live_projection.py","full_map_route.json","cloud_bridge.py")
node=Path("C:/Users/Administrator/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe")
checks={}
for name,command in (("serial_offline",[sys.executable,"-X","utf8",str(root/"test_shared_console.py")]),("ui_protocol",[str(node),str(root/"test_ui.js")])):
    result=subprocess.run(command,capture_output=True,text=True,encoding="utf-8")
    checks[name]=dict(exit_code=result.returncode,output=result.stdout+result.stderr)
    if result.returncode:raise RuntimeError(checks[name]["output"])
with zipfile.ZipFile(root/"assets.zip") as archive:
    packaged=archive.namelist()
    assert not any("log" in n or "pycache" in n or n.endswith(".key") for n in packaged)
evidence=dict(date="2026-10-27",original_sha256=digest(original),backup_sha256=digest(backup),
    original_unchanged=digest(original)=="cfc5135693d22409e0943370ebc40ae4f2c2d94ea4c8844273fac8fd44c37a90",
    reused_modules={n:digest(baseline/n)==digest(root/"assets"/n) for n in modules},
    lifecycle=json.loads((root/"lifecycle.json").read_text()),checks=checks,packaged=packaged,
    real_serial_opened=False,hardware_tested=False,public_network_verified=False,
    files={str(p.relative_to(root)):digest(p) for p in root.rglob("*") if p.is_file() and p.suffix in (".cs",".py",".html",".css",".js",".exe",".manifest",".ps1") and not any(x in p.parts for x in ("deps","__pycache__","logs"))})
assert evidence["original_unchanged"] and evidence["original_sha256"]==evidence["backup_sha256"]
assert all(evidence["reused_modules"].values())
assert evidence["lifecycle"]["ok"]
(root/"VALIDATION.json").write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding="utf-8")
print("Validation passed: original unchanged, backup identical, offline tests passed, final hashes recorded.")
