from http.client import IncompleteRead
from unittest import TestCase
from unittest.mock import patch

from tools.fas_energy_log import read_vehicle, segments


class EnergyHistoryTests(TestCase):
    def test_truncated_status_records_one_gap(self):
        with patch('tools.fas_energy_log.urlopen',side_effect=IncompleteRead(b'{"volt',30)):
            row=read_vehicle('A','http://127.0.0.1:5100')
        self.assertEqual((row[0],row[2],row[3]),('A','unknown',None))

    def test_charge_start_uses_onboard_elapsed_and_away_ends_at_parked_dock(self):
        rows = [(100,'charging',50),(105,'charging',50.1),(110,'away',None),
                (115,'away',None),(120,'near_dock',None),(125,'at_dock',None)]
        self.assertEqual(list(segments(rows)),[('charging',50,110,True),('away',110,125,True)])

    def test_gap_splits_unverified_operation_but_not_proven_continuous_charging(self):
        rows = [(100,'charging',50),(105,'unknown',None),(130,'charging',50.2),
                (135,'away',None),(140,'away',None),(150,'unknown',None),(155,'at_dock',None)]
        self.assertEqual(list(segments(rows)),[('charging',50,135,True),('away',135,140,False)])
