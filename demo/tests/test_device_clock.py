import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from device_clock import DeviceClock
class ClockTests(unittest.TestCase):
 def test_ten_minutes_clock_drift_does_not_become_false_backlog(self):
  c=DeviceClock();errors=[]
  for t in range(601):
   host=1000+t*1.0006+.015+.008*(1+math.sin(t*1.7))
   c.observe(t,host);mapped=c.map_time(t)
   if t>60:errors.append(host-mapped)
  self.assertLess(max(errors),.08);self.assertAlmostEqual(c.skew,.0006,delta=.0001)
 def test_fresh_heartbeat_does_not_hide_old_radio_sample(self):
  c=DeviceClock()
  for t in range(120):c.observe(t,1000+t*1.0006+.02)
  delay=(1000+119*1.0006+.02)-c.map_time(118.4)
  self.assertGreater(delay,.59)
 def test_temporary_host_stall_not_learned_as_drift(self):
  c=DeviceClock()
  for t in range(120):c.observe(t,1000+t*1.0006+.02+(.5 if 51<=t<=55 else 0))
  self.assertAlmostEqual(c.map_time(120),1000+120*1.0006+.02,delta=.04)
class PortCacheTests(unittest.TestCase):
 def test_scan_cached_without_exposing_mutable_result(self):
  import collection
  from unittest.mock import patch
  old=collection._ports_cache;collection._ports_cache=(0.,[])
  try:
   with patch.object(collection,'_scan_ws63_ports',return_value=['/dev/serial/by-id/test']) as scan:
    first=collection.ws63_ports();first.clear();second=collection.ws63_ports()
    self.assertEqual(second,['/dev/serial/by-id/test']);self.assertEqual(scan.call_count,1)
  finally:collection._ports_cache=old
if __name__=='__main__':unittest.main()
