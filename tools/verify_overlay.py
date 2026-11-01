import hashlib
import json
from pathlib import Path
import subprocess
import sys
from ament_index_python.packages import get_package_prefix

root=Path(sys.argv[1]).resolve();report={'packages':{},'libraries':{}}
for name in ('nav2_controller','nav2_rotation_shim_controller'):
    prefix=Path(get_package_prefix(name)).resolve()
    if not prefix.is_relative_to(root/'install'):raise RuntimeError('覆盖包未生效：'+name)
    report['packages'][name]=str(prefix)
    for p in (prefix/'lib').glob('*.so'):
        if 'not found' in subprocess.check_output(['ldd',str(p)],text=True):raise RuntimeError('缺少动态依赖：'+str(p))
        report['libraries'][str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'vendor_build_verified.json').write_text(json.dumps(report,indent=2));print('控制器编译与依赖检查完成。')
