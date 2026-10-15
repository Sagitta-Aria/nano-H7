"""Host save/apply regression tests; safe on Nano with no hardware access."""
import tempfile
import unittest
from pathlib import Path
from host.protocol import encode, decode
from host.webapp import Console

class HostAppliedLimitsTests(unittest.TestCase):
    def test_apply_is_confirmed_before_saved_settings_change(self):
        class Worker:
            def __init__(self): self.sent=[]
            def submit(self,body,tag=None): self.sent.append((body,tag)); return True
        with tempfile.TemporaryDirectory() as directory:
            c=Console(auxiliary_path=Path(directory)/'aux.json'); c.connection=Worker()
            c.device={'aux_control':'1','aux_live':'1'}; c.armed=True
            c.auxiliary={'5':{'active':'0','referenced':'1','pos':'0','min':'0','max':'8000'}}
            value=c.aux_settings.snapshot(); value['5']['max']=7200
            self.assertTrue(c.apply_auxiliary(value,'5')[0])
            body,tag=c.connection.sent[-1]
            self.assertEqual(body,'AUX CONFIG 5 0 7200')
            self.assertEqual(c.aux_settings.snapshot()['5']['max'],8000)
            c._reply(body,decode(encode(1,'OK cmd=AUX_STAT axis=5 min=0 max=7200 active=0 referenced=1 configured=1 pos=0')),tag)
            self.assertEqual(c.aux_settings.snapshot()['5']['max'],7200)

    def test_rejected_apply_keeps_saved_settings_and_reports_failure(self):
        class Worker:
            def __init__(self): self.sent=[]
            def submit(self,body,tag=None): self.sent.append((body,tag)); return True
        with tempfile.TemporaryDirectory() as directory:
            c=Console(auxiliary_path=Path(directory)/'aux.json'); c.connection=Worker()
            c.device={'aux_control':'1','aux_live':'1'}
            c.auxiliary={'5':{'active':'0','referenced':'1','pos':'0','min':'0','max':'8000'}}
            value=c.aux_settings.snapshot(); value['5']['max']=7200
            c.apply_auxiliary(value,'5');body,tag=c.connection.sent[-1]
            c._reply(body,decode(encode(1,'ERR code=AUX_CONFIG_RANGE_REFERENCE_OR_BUSY')),tag)
            self.assertEqual(c.aux_settings.snapshot()['5']['max'],8000)


if __name__=='__main__': unittest.main()
