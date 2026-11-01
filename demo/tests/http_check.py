"""Exercise an already running OFFLINE server; refuses live robot mode."""
import json
from pathlib import Path
import time
import urllib.request
import urllib.error

BASE = 'http://127.0.0.1:8890'


def get(path):
    return json.load(urllib.request.urlopen(BASE+path, timeout=3))


def main():
    state = get('/api/state')
    assert state['health']['mode'] == 'offline', 'Never run this check against the robot'
    token = state['control_token']

    def command(**data):
        req = urllib.request.Request(BASE+'/api/command', json.dumps(data).encode(),
              {'Content-Type':'application/json', 'X-Control-Token':token})
        return json.load(urllib.request.urlopen(req, timeout=3))

    request = urllib.request.Request(BASE+'/api/command', b'{"action":"stop"}',
                                     {'Content-Type':'application/json'})
    try:
        urllib.request.urlopen(request, timeout=3)
        raise AssertionError('Missing token was accepted')
    except urllib.error.HTTPError as e:
        assert e.code == 403
    for path in ('/', '/app.js', '/style.css', '/map.png'):
        data = urllib.request.urlopen(BASE+path, timeout=3).read()
        assert len(data) > 100
        if path.endswith('.png'):
            assert data.startswith(b'\x89PNG\r\n\x1a\n')
    command(action='stop')
    command(action='initial_dock')
    command(action='confirm')
    points = [{'id':'http-a','name':'A','x':.25,'y':-.5,'yaw':0,'wait':.3,'enabled':True},
              {'id':'http-b','name':'B','x':.25,'y':-1,'yaw':0,'wait':.2,'enabled':True}]
    command(action='save_points',points=points)
    before = len(get('/api/state')['events'])
    command(action='start',auto_return=True)
    phases = []
    deadline = time.monotonic()+25
    while time.monotonic() < deadline:
        state = get('/api/state')
        if not phases or phases[-1] != state['phase']:
            phases.append(state['phase'])
        if state['phase'] in ('docked','failed'):
            break
        time.sleep(.1)
    assert state['phase'] == 'docked', state
    assert state['health']['charging'] is True
    new_events = state['events'][:len(state['events'])-before]
    assert sum('停留结束' in e['text'] for e in new_events) == 2
    assert 'departing' in phases and 'dwelling' in phases and 'docking' in phases
    proof = {'scope':'Offline HTTP workflow; simulated robot, not physical navigation',
             'result':state['phase'],'phases':phases,'dwell_completions':2,
             'unauthorized_post':403,'static_assets':'passed','events':new_events}
    folder = Path(__file__).resolve().parents[1]/'验证记录'
    folder.mkdir(exist_ok=True)
    (folder/'http_workflow.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2),encoding='utf-8')
    # Leave two convenient rehearsal points. Real mode uses its own empty data/ros.
    command(action='stop')
    command(action='save_points',points=[
        {'id':'sample-1','name':'走廊点 1（演练）','x':.25,'y':-1.5,'yaw':0,'wait':10,'enabled':True},
        {'id':'sample-2','name':'走廊点 2（演练）','x':.25,'y':-3,'yaw':0,'wait':10,'enabled':True}])
    command(action='initial_dock')
    print(json.dumps(proof,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
