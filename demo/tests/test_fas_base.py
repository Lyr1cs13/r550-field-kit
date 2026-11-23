from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from unittest import TestCase

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
from fas_base import Canceled, Station, WS_IDS, c5_word


class Receiver:
    def __init__(self, radio):
        self.radio=radio
        self.log=None
        self.commands=[]
        self.session='-'
        self.state=0

    def request(self, command, predicate, *args, **kwargs):
        self.commands.append(command)
        verb,*params=command.split(',')
        if verb in ('WSGET','WSSTOP','WSONCE'):
            sources={'WSGET':'query','WSSTOP':'stopped','WSONCE':'one_shot'}
            if params and verb=='WSONCE':
                self.state=int(params[0])
            row=f'WS_STATUS,oneshot_api=1,running=0,node={self.state},attempt=0,source={sources[verb]}'
        elif verb=='WSROUND':
            row='WS_ROUND_CONFIG,threshold10=60'
        elif verb=='SESSIONSTATUS':
            status='legacy' if self.session=='-' else 'streaming'
            row=f'Y,{status},{self.session},0,0,0,0,-,-,query'
        elif verb=='SESSIONARM':
            self.session=params[0]
            row=f'Y,armed,{self.session},0,0,0,0,-,-,armed'
        elif verb=='CLIENTCFG':
            row='W,whitelist,selection_applied'
        elif verb=='CFG':
            row='S,10,10,16,57,1000,nested,usb'
        elif verb=='GPIO24CFG':
            main,sub=params[-2:]
            self.state=int(main) | (int(sub)<<12)
            assert not predicate(f'H,sequence,sequence,1,1,{main},{sub},gpio24')
            row=f'A,3,{sub},3,{sub},acked,uart1,100,gpio24'
        elif verb=='SESSIONSTART':
            row=f'Y,streaming,{self.session},0,0,0,0,-,-,started'
        elif verb=='SESSIONSTOP':
            row=f'Y,stopped,{self.session},0,0,0,0,host_stop,{self.session},stopped'
            self.session='-'
        else:
            raise AssertionError(command)
        assert predicate(row), (command,row)
        return row

    def lines(self, deadline, cancel):
        if self.radio=='ws':
            n1,n2=divmod(self.state,64)
            yield f'#STATE,node={self.state},attempt=1,n1={n1},n2={n2}'
            yield f'RF_STATE,state={self.state},valid=1,count_a=10,sum_a=-600,count_b=10,sum_b=-660'
            yield f'WS_STATUS,running=0,node={self.state},attempt=1,source=one_shot_done'
        else:
            for epoch in range(22):
                for suffix,rssi in (('01',-61),('02',-68)):
                    yield f'C,{epoch},02:00:00:00:00:{suffix},{rssi},{self.state},AAAA'


class BaseStationTests(TestCase):
    def test_specified_state_finishes_once_and_receipt_survives_restart(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)
            ws,c5=Receiver('ws'),Receiver('c5')
            links={'sle':WS_IDS,'c5':{'A':'02:00:00:00:00:01','B':'02:00:00:00:00:02'}}
            station=Station(ws,c5,path,links)
            station.prepare()
            def request(trial,state):
                return dict(protocol='FAS_PAIR_1',run_id='run',trial_id=trial,state_id=state,
                            radios={'sle':list(divmod(state,64)),'c5':c5_word(state)},tx=links,
                            min_samples=5,window_s=.2,
                            cars={c:dict(map_id='map',frame='map',point_id=c,pose=[1,2,0]) for c in ('A','B')})
            for state in (200,201):
                query=request(f'trial_{state}',state)
                station.start(query)
                self.assertTrue(station.done.wait(2.))
                record=station.receipt(query['trial_id'])
                self.assertEqual(record['status'],'complete',record)
                self.assertEqual(record['data']['sle/main']['rssi_dbm'],{'A':-60.,'B':-66.})
                self.assertEqual(record['data']['c5/main']['rssi_dbm'],{'A':-61.,'B':-68.})
                self.assertEqual(station.start(query),record)
            self.assertEqual([x for x in ws.commands if x.startswith('WSONCE')],['WSONCE,200,205,5','WSONCE,201,205,5'])
            partial=path/'partial'
            partial.mkdir()
            station.save(partial/'receipt.json',dict(request('partial',202),status='running'))
            restarted=Station(ws,c5,path,links)
            restarted.prepare()
            self.assertEqual(restarted.receipt('trial_200')['status'],'complete')
            self.assertEqual(restarted.receipt('partial')['status'],'incomplete')

            original=ws.lines
            def interrupted(deadline,cancel):
                if not cancel.wait(1.):
                    raise AssertionError('capture was not canceled')
                raise Canceled()
                yield  # Keep the same iterator contract as the serial receiver.

            ws.lines=interrupted
            query=request('cancel_202',202)
            restarted.start(query)
            self.assertEqual(restarted.abort(query),{'status':'idle'})
            self.assertEqual(restarted.receipt('cancel_202')['status'],'canceled')
            self.assertEqual(ws.commands[-1],'WSSTOP')
            ws.lines=original
            restarted.start(request('retry_202',202))
            self.assertTrue(restarted.done.wait(2.))
            self.assertEqual(restarted.receipt('retry_202')['status'],'complete')

    def test_mapping_preserves_main_card_as_fast_axis(self):
        self.assertEqual([c5_word(n) for n in (0,1,64,4095)],[0,65,65<<12,0xffffff])
        self.assertEqual(len({c5_word(n) for n in range(4096)}),4096)
