#!/usr/bin/env python3
"""Fast source, scene, link and accidental-credential checks; no robot access."""
import ast
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    errors=[];sources=[]
    for p in ROOT.rglob('*'):
        rel=p.relative_to(ROOT)
        if not p.is_file() or any(c in ('.git','state','maps','data','__pycache__','vendor_nav2_overlay','.venv') for c in rel.parts):continue
        if p.suffix=='.py':
            try:ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(rel));sources.append(p)
            except SyntaxError as e:errors.append(str(e))
        if p.suffix in ('.md','.py','.sh','.yaml','.json','.env','.ps1'):
            text=p.read_text(encoding='utf-8-sig')
            if re.search(r'-----BEGIN (OPENSSH|RSA|EC) PRIVATE KEY-----|ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,}',text):errors.append('凭据内容：'+str(rel))
        if p.suffix=='.md':
            for target in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
                if '://' not in target and not target.startswith('#') and not (p.parent/target.split('#')[0]).exists():errors.append('链接不存在：'+str(rel)+' → '+target)
    sys.path.insert(0,str(ROOT/'demo'))
    from map_grid import MapGrid
    from survey_path import checked_path
    g=MapGrid(ROOT/'scenes/demo-room');route=json.loads((g.directory/'survey_route.json').read_text())
    if route['map_id']!=g.map_id:errors.append('示例路线 map_id 不匹配')
    for p in route['points']:checked_path(g,p['survey_path'],[p['x'],p['y']])
    if sys.platform!='win32':
        for p in [ROOT/'kit',*ROOT.rglob('*.sh')]:
            if any(c in ('.git','vendor_nav2_overlay') for c in p.relative_to(ROOT).parts):continue
            r=subprocess.run(['bash','-n',str(p)],capture_output=True,text=True)
            if r.returncode:errors.append(r.stderr)
    if errors:raise SystemExit('\n'.join(errors))
    print(f'通过：{len(sources)} 个 Python 源文件、示例场景、路径、文档链接与凭据检查。')


if __name__=='__main__':main()
