"""Record read-only vehicle energy telemetry on the FAS controller host."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from http.client import IncompleteRead
import json
import math
from pathlib import Path
import sqlite3
import time
from urllib.request import urlopen


INTERVAL = 5.


def read_vehicle(car, endpoint):
    started = time.time()
    try:
        with urlopen(endpoint.rstrip('/')+'/api/fas/state', timeout=2.) as response:
            status = json.load(response)
        if status['fas']['car'] != car:
            raise ValueError('车辆身份不符')
        health = status['health']
        stamp = (started+time.time())/2
        elapsed = health.get('charging_elapsed_s')
        if health['charging_stable'] and isinstance(elapsed,(int,float)) and math.isfinite(elapsed) and elapsed >= 0:
            mode, charge_start = 'charging', stamp-elapsed
        else:
            pose = health.get('pose')
            if not health.get('localization_confirmed') or not pose:
                mode = 'unknown'
            elif status['phase']=='departing' and not health['stationary'] and not health.get('charge_contact_live'):
                mode = 'away'
            else:
                distance = math.dist(pose[:2],status['dock'][:2])
                mode = 'away' if distance > .22 else 'at_dock' if health['stationary'] else 'near_dock'
            charge_start = None
        return (car, stamp, mode, charge_start, health.get('voltage'),health.get('current'),
                status['phase'],bool(status['fas'].get('owner')))
    except (OSError,IncompleteRead,ValueError,KeyError,TypeError):
        return (car,time.time(),'unknown',None,None,None,None,False)


def segments(rows):
    """Keep telemetry gaps explicit and preserve the onboard charge start time."""
    previous = None
    disconnected = False
    for stamp, mode, charge_start in rows:
        if mode == 'unknown':
            disconnected = True
            continue
        if previous is not None:
            kind, start, last, complete = previous
            gap = disconnected or stamp-last > INTERVAL*2.5
            if mode == 'near_dock' and kind == 'away' and not gap:
                mode = 'away'
            # A fresh onboard timer can prove uninterrupted charging across a host gap.
            same_charge = (kind == mode == 'charging' and abs(charge_start-start) <= 2.)
            if same_charge or (not gap and kind == mode and mode != 'charging'):
                previous = (kind,start,stamp,complete)
                disconnected = False
                continue
            if kind in ('charging','away'):
                known_end = not gap and mode != 'unknown'
                yield kind,start,stamp if known_end else last,complete and known_end
        else:
            kind, gap = 'unknown', True
        previous = (mode,charge_start if mode=='charging' else stamp,stamp,
                    mode=='charging' or (not gap and kind!='unknown'))
        disconnected = False
    if previous and previous[0] in ('charging','away'):
        kind,start,last,complete=previous
        yield kind,start,last,False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database',type=Path,required=True)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--summary',action='store_true')
    args=parser.parse_args()
    if args.summary:
        with sqlite3.connect(args.database.resolve().as_uri()+'?mode=ro',uri=True) as db:
            for car in ('A','B'):
                rows=db.execute('SELECT stamp,mode,charge_start FROM samples WHERE car=? ORDER BY stamp',(car,))
                for kind,start,end,complete in segments(rows):
                    fmt=lambda stamp:datetime.fromtimestamp(stamp).astimezone().isoformat(timespec='seconds')
                    print(json.dumps(dict(car=car,mode=kind,start=fmt(start),end=fmt(end),
                                          seconds=round(end-start,1),complete=complete)))
        return
    if args.config is None:
        parser.error('--config is required to record samples')
    endpoints=json.loads(args.config.read_text(encoding='utf-8-sig'))['cars']
    args.database.parent.mkdir(parents=True,exist_ok=True)
    with sqlite3.connect(args.database,timeout=5.) as db, ThreadPoolExecutor(max_workers=2) as pool:
        db.execute('''CREATE TABLE IF NOT EXISTS samples (
            car TEXT, stamp REAL, mode TEXT, charge_start REAL, voltage REAL,
            current REAL, phase TEXT, claimed INTEGER, PRIMARY KEY(car,stamp))''')
        while True:
            now=time.monotonic()
            pending=[pool.submit(read_vehicle,car,endpoints[car]) for car in ('A','B')]
            with db:
                db.executemany('INSERT INTO samples VALUES (?,?,?,?,?,?,?,?)',[job.result() for job in pending])
            time.sleep(max(0.,INTERVAL-(time.monotonic()-now)))


if __name__=='__main__':
    main()
