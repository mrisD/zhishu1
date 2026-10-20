import json
from pathlib import Path
import tempfile
import unittest
from pallet_run_contract import manifest,verify

class Contract(unittest.TestCase):
    def test_content_or_link_target_change_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);a=d/'a';b=d/'b';link=d/'binary';contract=d/'contract.json'
            a.write_bytes(b'validated binary');b.write_bytes(b'other version');link.symlink_to(a)
            contract.write_text(json.dumps({'files':manifest([link])}))
            self.assertTrue(verify(contract))
            link.unlink();link.symlink_to(b)
            with self.assertRaises(RuntimeError):verify(contract)
    def test_same_size_configuration_change_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);p=d/'config';c=d/'contract';p.write_text('speed=1')
            c.write_text(json.dumps({'files':manifest([p])}));p.write_text('speed=2')
            with self.assertRaises(RuntimeError):verify(c)

if __name__=='__main__':unittest.main()
