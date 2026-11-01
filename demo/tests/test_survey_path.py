import unittest,sys,json,math,ast,threading
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_grid import MapGrid
from survey_path import checked_path,advance_index,ahead
class SurveyPathTests(unittest.TestCase):
 def test_parallel_return_does_not_prune_outbound(self):
  p=[[i*.02,0.] for i in range(151)]+[[3-i*.02,.2] for i in range(151)]
  self.assertLess(advance_index(p,20,[.41,.19,0]),75)
  self.assertGreater(advance_index(p,200,[1.9,.2,0]),200)
 def test_reject_jump_or_wrong_endpoint(self):
  g=SimpleNamespace(validate=lambda *a:None)
  for path,end in [([[0,0],[1,1]],[1,1]),([[0,0],[.02,0]],[.2,0])]:
   with self.assertRaises(ValueError):checked_path(g,path,end)
 def test_saved_parallel_route_clearance_and_hazard(self):
  demo=Path(__file__).resolve().parents[1];r=json.loads((demo/'assets/survey_route.json').read_text(encoding='utf-8'));p=r['points'][0]
  path=checked_path(MapGrid(demo/'assets'),p['survey_path'],[p['x'],p['y']])
  self.assertEqual(len(r['points']),2)
  self.assertLess(math.dist(path[-1],[4.8,0]),.03)
  self.assertTrue(any(abs(y+7.5)<.01 and 1<x<3 for x,y in path))
  self.assertTrue(any(abs(y+7.1)<.01 and 1<x<3 for x,y in r['points'][1]['survey_path']))
 def method(self,name):
  code=ast.parse((Path(__file__).resolve().parents[1]/'survey_ros.py').read_text(encoding='utf-8-sig'));cls=next(n for n in code.body if isinstance(n,ast.ClassDef))
  node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name);m=ast.Module(body=[node],type_ignores=[]);ns={};exec(compile(ast.fix_missing_locations(m),'test','exec'),ns);return ns[name]
 def test_stop_wins_over_late_validation(self):
  op={'cancel':True,'status':'validating'};backend=SimpleNamespace(lock=threading.RLock(),ops={'a':op})
  self.method('survey_validated')(backend,None,'a');self.assertEqual(op['status'],'canceled')
 def test_stop_cancels_accepted_follow_handle(self):
  calls=[];handle=SimpleNamespace(accepted=True,cancel_goal_async=lambda:calls.append('cancel'),get_result_async=lambda:SimpleNamespace(add_done_callback=lambda f:None))
  op={'cancel':True};backend=SimpleNamespace(lock=threading.RLock(),ops={'a':op})
  self.method('survey_accepted')(backend,SimpleNamespace(result=lambda:handle),'a');self.assertEqual(calls,['cancel'])
if __name__=='__main__':unittest.main()
