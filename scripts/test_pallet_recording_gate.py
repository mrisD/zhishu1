import copy
import unittest
from record_pallet_after_verified_nine import ORDER, verification_error

class RecordingGate(unittest.TestCase):
    def valid(self):
        return dict(passed=True,process_admittance_passed=True,all_nine_initially_physical=True,
                    single_continuous_scene=True,completed=ORDER.copy(),failure=None,
                    evidence=[dict(index=i,process_admittance=dict(passed=True)) for i in ORDER])
    def test_full_evidence(self):
        self.assertIsNone(verification_error(self.valid()))
    def test_incomplete_or_failed_runs_never_release(self):
        for field,value in [('completed',ORDER[:-1]),('failure','guard stop'),('diagnostic_only',True),
                            ('third_row_diagnostic',True),('all_nine_initially_physical',False),
                            ('resumed',True),('stitched',True),('scene_restarts',1),('recording_restarts',1)]:
            with self.subTest(field=field):
                r=self.valid();r[field]=value;self.assertIsNotNone(verification_error(r))
    def test_missing_reordered_or_failed_audits(self):
        for mode in range(3):
            r=copy.deepcopy(self.valid())
            if mode==0:r['evidence'].pop()
            if mode==1:r['evidence'].reverse()
            if mode==2:r['evidence'][4]['process_admittance']['passed']=False
            self.assertIsNotNone(verification_error(r))

if __name__=='__main__':unittest.main()
