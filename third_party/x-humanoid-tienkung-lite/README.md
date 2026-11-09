# TienKung Lite（天工 Lite）上游文件

- 来源：https://github.com/Open-X-Humanoid/TienKung_URDF （`lite_urdf_publish/`）
- 许可：OpenAtom Open Hardware License 1.0（见 `LICENSE`）
- 入库内容：`urdf/humanoid_publish.urdf`、`LICENSE`、上游 `README.upstream.md`
- **STL 网格没有入库**（22 个文件共 145 MB），转换后的 GLB 在 `assets/tienkung/`（62 MB）。

重新拉取原始 STL：

```bash
cd third_party/x-humanoid-tienkung-lite
mkdir -p meshes && python3 - <<'PY'
import json, urllib.request
tree = json.load(urllib.request.urlopen(
    "https://api.github.com/repos/Open-X-Humanoid/TienKung_URDF/git/trees/main?recursive=1"))
for t in tree["tree"]:
    p = t["path"]
    if t["type"] == "blob" and p.startswith("lite_urdf_publish/meshes/") and p.lower().endswith(".stl"):
        urllib.request.urlretrieve(
            "https://raw.githubusercontent.com/Open-X-Humanoid/TienKung_URDF/main/" + p,
            "meshes/" + p.rsplit("/", 1)[-1])
PY
python3 tools/robot/stl2glb.py third_party/x-humanoid-tienkung-lite/meshes assets/tienkung
```
