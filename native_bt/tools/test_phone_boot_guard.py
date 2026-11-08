"""Offline pause-guard lifecycle and reset-site checks; no hardware."""
import ast,importlib.util,pathlib,types,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('phone_boot_guard',ROOT/'tools/phone_boot_guard.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)

class Fake:
    def __init__(self):self.now=0.;self.calls=[];self.bad=None;self.sleeps=[];self.nonce='a'*32;self.pid=100
    def sleep(self,n):self.sleeps.append(n);self.now+=n
    def run(self,args,**kwargs):
        assert kwargs['check'] and kwargs['timeout']==10 and kwargs['capture_output']
        assert args[:3]==['adb','-s','9125299103B7']
        cmd=args[3:];self.calls.append(cmd);out=''
        if cmd==['get-state']:out='offline' if self.bad=='offline' else 'device'
        elif cmd[:2]==['shell','-T']:
            assert 'nohup sh -c ' in cmd[2] and 'sleep 14' in cmd[2] and 'dispatch pause' in cmd[2]
            assert 'dispatch play' not in cmd[2]
            self.pid+=1;out='' if self.bad=='ack' else 'BDK_GUARD_PID='+str(self.pid)+'\n'
        elif cmd[:2]==['exec-out','cat']:
            out=('wrong process' if self.bad=='identity' else self.nonce) if cmd[2].startswith('/proc/') else ('1' if self.bad=='marker' else '0')
        elif cmd==['shell','dumpsys','media_session']:
            out='package=com.google.android.youtube\n state=PlaybackState {state='+('PLAYING' if self.bad=='playing' else 'PAUSED')+'(2)}'
        return types.SimpleNamespace(stdout=out,stderr='',returncode=0)
    def make(self):return guard.PhoneBootPauseGuard('9125299103B7',adb='adb',run=self.run,clock=lambda:self.now,sleep=self.sleep,nonce=self.nonce)
    def pauses(self):return sum(c==['shell','cmd','media_session','dispatch','pause'] for c in self.calls)

class GuardTests(unittest.TestCase):
    def test_independent_and_completed(self):
        f=Fake();g=f.make();g.precheck();g.arm()
        self.assertFalse(f.sleeps);self.assertEqual(len(g.records),1)
        f.now=5.;r=g.finish();self.assertEqual(f.sleeps,[9.25])
        self.assertTrue(r['post_boot_pause_verified']);self.assertTrue(r['guards'][0]['completed'])
        self.assertEqual(f.pauses(),3)
        rm=[c for c in f.calls if c[:3]==['shell','rm','-f']]
        self.assertEqual([c[3] for c in rm],[g.records[0]['marker']])
        n=len(f.calls);g.finish();self.assertEqual(n,len(f.calls))
    def test_two_owned_resets_bounded(self):
        f=Fake();g=f.make();g.arm();f.now=1.;g.arm()
        self.assertNotEqual(g.records[0]['marker'],g.records[1]['marker'])
        with self.assertRaises(RuntimeError):g.arm()
        self.assertTrue(g.finish()['post_boot_pause_verified'])
    def test_failures_pause_without_false_success(self):
        for bad in ['marker','playing']:
            with self.subTest(bad=bad):
                f=Fake();g=f.make();g.arm();f.bad=bad
                with self.assertRaises(RuntimeError):g.finish()
                self.assertFalse(g.finished);self.assertGreaterEqual(f.pauses(),2)
    def test_no_reset_before_ack_and_identity(self):
        for bad in ['offline','ack','identity']:
            with self.subTest(bad=bad):
                f=Fake();g=f.make();f.bad=bad
                with self.assertRaises(RuntimeError):
                    if bad=='offline':g.precheck()
                    else:g.arm()
                self.assertFalse(g.records)
    def test_invalid_inputs(self):
        f=Fake()
        for serial in ['','abc; reboot','x y','x\n']:
            with self.assertRaises(ValueError):guard.PhoneBootPauseGuard(serial,adb='adb',run=f.run)
        with self.assertRaises(ValueError):guard.PhoneBootPauseGuard('9125299103B7',adb='adb',nonce='unsafe/path')
        self.assertFalse(f.calls)
    def test_every_persistent_reset_is_guarded(self):
        tree=ast.parse((ROOT/'tools/deploy_audio_trial.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        found=[]
        for parent in ast.walk(main):
            for field in ['body','orelse','finalbody']:
                children=getattr(parent,field,None)
                if not isinstance(children,list):continue
                for i,node in enumerate(children):
                    if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='hard_reset':
                        self.assertGreater(i,0)
                        previous=children[i-1]
                        self.assertIsInstance(previous,ast.If);self.assertEqual(ast.unparse(previous.test),'phone_guard')
                        self.assertEqual(ast.unparse(previous.body[0]),'phone_guard.arm()')
                        found.append(node.lineno)
        self.assertEqual(len(found),2)
        text=ast.unparse(main)
        self.assertLess(text.index('phone_guard.precheck()'),text.index('esptool.detect_chip'))
        self.assertIn("'.phone-guard.failure.json'",text)

if __name__=='__main__':unittest.main(verbosity=2)
