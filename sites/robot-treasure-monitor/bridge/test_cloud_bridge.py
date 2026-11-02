import importlib.util,unittest
from pathlib import Path
from unittest.mock import patch
spec=importlib.util.spec_from_file_location('cloud_bridge',Path(__file__).with_name('cloud_bridge.py'));bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)
class Commands(unittest.TestCase):
    def cmd(self,**kw):return dict(id='one',session='current',issuedAt=1000,action='disconnect',**kw)
    def test_current_disconnect(self):
        with patch.object(bridge,'call',return_value={}) as post:
            ack=bridge.execute(self.cmd(),'http://127.0.0.1:8766','current',None,1001)
            self.assertTrue(ack['result']['ok']);self.assertEqual(post.call_args.args[1]['action'],'disconnect')
    def test_expired_restarted_and_duplicate_never_execute(self):
        with patch.object(bridge,'call') as post:
            for session,seen,now in [('different',None,1001),('current','one',1001),('current',None,17000),('current',None,900)]:
                self.assertIsNone(bridge.execute(self.cmd(),'http://127.0.0.1:8766',session,seen,now))
            post.assert_not_called()
    def test_unknown_action_never_executes(self):
        command=self.cmd();command['action']='motor-forward'
        with patch.object(bridge,'call') as post:
            self.assertIsNone(bridge.execute(command,'http://127.0.0.1:8766','current',None,1001));post.assert_not_called()
if __name__=='__main__':unittest.main()
