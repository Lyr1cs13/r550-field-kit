#!/usr/bin/env python3
"""Save the RDK's collector URL after checking the service and the shared token."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'demo'))
from c5_station import CONFIG, TOKEN, CollectorLink, read_token, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('url')
    args = parser.parse_args()
    try:
        link = CollectorLink(args.url, read_token(TOKEN))
        plan = link.ready()
        write_json(CONFIG, {'url': link.url})
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'C5 采集端配置未保存：{exc}\n')
    print(f'C5 采集端已连接 ({plan})。在巡航面板启用每站 C5 采集。')


if __name__ == '__main__':
    main()
