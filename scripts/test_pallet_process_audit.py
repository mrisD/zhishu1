import unittest
from audit_pallet_process_admittance import audit

class ProcessAudit(unittest.TestCase):
    def content(self):
        stages=['empty long-finger coordinated approach','right-arm pre-extraction box',
                'full-transfer pick box','coordinated level tote posture','large-bin insertion box']
        return '\n'.join('PROCESS_ADMITTANCE_END stage='+s+' cycles=4 max_abs_offset=0.0001 corrected_points=8 projected_points=0' for s in stages)+'\nPROCESS_ADMITTANCE_CARRY_SAMPLE\nFloor support verified BEFORE opening\nPhysical large-bin placement verified'
    def test_enable_only_is_not_a_pass(self):
        self.assertFalse(audit('PROCESS_ADMITTANCE_BEGIN','box',0)['passed'])
    def test_requires_actual_correction_and_carry(self):
        self.assertTrue(audit(self.content(),'box',0)['passed'])
        self.assertFalse(audit(self.content().replace('corrected_points=8','corrected_points=0'),'box',0)['passed'])
        self.assertFalse(audit(self.content().replace('PROCESS_ADMITTANCE_CARRY_SAMPLE',''),'box',0)['passed'])
    def test_late_failure_rejects_complete_markers(self):
        self.assertFalse(audit(self.content()+'\n[FATAL]','box',0)['passed'])

if __name__=='__main__': unittest.main()
