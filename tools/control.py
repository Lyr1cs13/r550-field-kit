#!/usr/bin/env python3
"""Small command-line client for the same controls as the browser panel."""
import argparse
import json
import urllib.request


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['status','stop','pause','resume','confirm','initial_dock','return','start']);p.add_argument('--host',default='127.0.0.1');a=p.parse_args()
    base=f'http://{a.host}:8890'
    s=json.load(urllib.request.urlopen(base+'/api/state',timeout=4))
    if a.action!='status':
        req=urllib.request.Request(base+'/api/command',data=json.dumps({'action':a.action}).encode(),headers={'Content-Type':'application/json','X-Control-Token':s['control_token']},method='POST')
        s=json.load(urllib.request.urlopen(req,timeout=8))
    print(json.dumps({k:s[k] for k in ('phase','message','health')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
