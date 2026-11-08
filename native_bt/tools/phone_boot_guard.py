"""Pause-only phone guard anchored to an explicitly requested board reset."""
import re,shlex,shutil,subprocess,time,uuid

class PhoneBootPauseGuard:
    def __init__(self,serial,*,adb=None,run=subprocess.run,clock=time.monotonic,sleep=time.sleep,nonce=None):
        if not re.fullmatch(r'[A-Za-z0-9._:-]{1,80}',serial):raise ValueError('Invalid ADB serial')
        self.serial=serial;self.adb=adb or shutil.which('adb')
        if not self.adb:raise RuntimeError('ADB is required for the requested phone guard')
        self.run=run;self.clock=clock;self.sleep=sleep
        self.nonce=nonce or uuid.uuid4().hex
        if not re.fullmatch(r'[0-9a-f]{32}',self.nonce):raise ValueError('Invalid guard nonce')
        self.records=[];self.finished=False
    def command(self,args):
        return self.run([self.adb,'-s',self.serial,*args],capture_output=True,text=True,check=True,timeout=10)
    def precheck(self):
        if self.command(['get-state']).stdout.strip()!='device':raise RuntimeError('Approved phone is not ready')
        self.command(['shell','cmd','media_session','dispatch','pause'])
    def arm(self):
        # A bounded device-side child survives the launcher's ADB connection.
        # Each reset gets its own nonce marker; no background loop is installed.
        if len(self.records)>=2:raise RuntimeError('Unexpected third board reset')
        marker='/data/local/tmp/bdk_boot_pause_'+self.nonce+'_'+str(len(self.records))+'.done'
        inner='sleep 14; cmd media_session dispatch pause; code=$?; printf \'%s\\n\' "$code" > '+shlex.quote(marker)
        script='nohup sh -c '+shlex.quote(inner)+' </dev/null >/dev/null 2>&1 & printf \'BDK_GUARD_PID=%s\\n\' "$!"'
        result=self.command(['shell','-T',script])
        matches=re.findall(r'^BDK_GUARD_PID=([1-9][0-9]*)\s*$',result.stdout,re.M)
        if len(matches)!=1:raise RuntimeError('Phone guard did not acknowledge its process')
        pid=int(matches[0]);self.command(['shell','kill','-0',str(pid)])
        if self.nonce not in self.command(['exec-out','cat','/proc/'+str(pid)+'/cmdline']).stdout:
            raise RuntimeError('Phone guard process identity mismatch')
        self.records.append({'pid':pid,'marker':marker,'armed_at':self.clock(),'pause_after_seconds':14,'completed':False})
        self.finished=False
    def finish(self):
        if self.finished:return self.report()
        # Normally boot capture already consumed these fourteen seconds.
        # Waiting here is host tooling only, never an ESP32 callback.
        self.command(['shell','cmd','media_session','dispatch','pause'])
        try:
            for record in self.records:
                remaining=14.25-(self.clock()-record['armed_at'])
                if remaining>0:self.sleep(remaining)
                result=self.command(['exec-out','cat',record['marker']])
                if result.stdout.strip()!='0':raise RuntimeError('Independent phone pause failed')
                record['completed']=True
        finally:
            self.command(['shell','cmd','media_session','dispatch','pause'])
        text=self.command(['shell','dumpsys','media_session']).stdout
        section=text.split('package=com.google.android.youtube',1)
        if len(section)==2:
            state=re.search(r'state=PlaybackState \{state=([A-Z_]+)',section[1])
            if state and state[1]=='PLAYING':raise RuntimeError('YouTube still reports playing after boot pause')
        for record in self.records:
            # Exact task-created paths only; no wildcard or recursive deletion.
            self.command(['shell','rm','-f',record['marker']])
        self.finished=True
        return self.report()
    def report(self):
        return {'serial':self.serial,'guarded_resets':len(self.records),'pause_only':True,
                'independent_phone_process':True,'post_boot_pause_verified':self.finished,
                'guards':[{'pid':r['pid'],'pause_after_seconds':14,'completed':r['completed']} for r in self.records]}
