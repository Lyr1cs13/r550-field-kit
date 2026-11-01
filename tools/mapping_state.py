#!/usr/bin/env python3
"""Initialize fresh mapping state at a physically seated charging dock."""
import json
from pathlib import Path
import shutil
import time

ROOT=Path(__file__).resolve().parents[1]


def prepare():
    state=ROOT/'state/mapping'
    if state.exists():shutil.move(str(state),str(ROOT/'state'/time.strftime('mapping_before_%Y%m%d_%H%M%S')))
    state.mkdir(parents=True)
    dock={'calibrated':True,'robot_pose':{'x':0.,'y':0.,'yaw':0.},'charger_approach':{'p_x':.6,'p_y':0.,'orien_z':-.1305261922,'orien_w':.9914448614},'departure_distance':.3}
    (state/'dock_start.json').write_text(json.dumps(dock,indent=2))
    (state/'threshold_avoid.json').write_text(json.dumps({'additional_keepout_rectangles':[]}))
    (state/'threshold_keepout.pgm').write_bytes(b'P5\n40 40\n255\n'+bytes([254])*1600)
    (state/'threshold_keepout.yaml').write_text('image: threshold_keepout.pgm\nresolution: 0.05\norigin: [-1, -1, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(state)


if __name__=='__main__':prepare()
