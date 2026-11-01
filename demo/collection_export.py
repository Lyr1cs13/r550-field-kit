"""Join original RSSI rows with immutable arrival/scan bindings, offline."""
import csv
import io
import json
from pathlib import Path
import zipfile

FIELDS = 'scan,node,p1,p2,p3,p4,p5,elapsed_ms,bytes,packets,bps,rssi_count,rssi_avg,rssi_min,rssi_max,status'.split(',')
PREFIX = ['session','visit_id','point_id','point_number','map_id','frame_id',
          'actual_x_m','actual_y_m','actual_yaw_rad','target_x_m','target_y_m','target_yaw_rad']


def merged_csv(data):
    records=[json.loads(line) for line in data.decode('utf-8').splitlines() if line.strip()]
    arrivals={r['visit_id']:r for r in records if r['event']=='arrival'}
    wire={(r['run'],r['visit']):arrivals[r['visit_id']] for r in records
          if r['event']=='wire_arrival' and r['visit_id'] in arrivals}
    output=io.StringIO();writer=csv.writer(output);writer.writerow(PREFIX+FIELDS)
    binding=None
    for r in records:
        if r['event']!='device': continue
        line=r['line']
        if line.startswith('+RDKBIND:'):
            run,visit,scan=map(int,line.split(':',1)[1].split(','))
            binding=(scan,wire.get((run,visit)))
        elif line.startswith('[scan_csv] '):
            values=next(csv.reader([line[len('[scan_csv] '):]]))
            if values[0]=='scan': continue
            if len(values)!=len(FIELDS) or not binding or int(values[0])!=binding[0] or not binding[1]:
                raise ValueError('存在无法关联到地图点位的 RSSI 记录，保留原始日志供排查')
            a=binding[1]
            writer.writerow([a[k] for k in PREFIX[:6]]+a['actual_pose']+a['target_pose']+values)
    return output.getvalue().encode('utf-8-sig')


def bundle(directory, data):
    directory=Path(directory);out=io.BytesIO()
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('journal.jsonl',data)
        try: z.writestr('rssi_with_positions.csv',merged_csv(data))
        except (ValueError,KeyError) as exc: z.writestr('EXPORT_ERROR.txt',str(exc))
        for p in (directory/'maps').rglob('*'):
            if p.is_file(): z.write(p,p.relative_to(directory).as_posix())
        z.writestr('README.txt','Coordinates: ROS map frame, metres, yaw radians, counterclockwise about +Z.\n'
                   'Actual pose is the localization estimate of base_footprint, not surveyed ground truth.\n'
                   'Map and keepout are versioned by full SHA256 map_id. Load map.yaml with ROS map_server; view in RViz.\n'
                   'CSV contains per-arm-node RSSI rows; point_id is a car station, node is an arm pose.\n'
                   'Use journal.jsonl for scan/run/visit correlation, quality and cancellation records.\n')
    return out.getvalue()
