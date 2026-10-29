"""Remove only the disposable Firefox profile created by the project audio test."""
import json
import os
from pathlib import Path
import shutil

root=Path.home()/'snap/firefox/common/native_bt_audio'
profile=root/'profile'
assert root.resolve()==root and profile.resolve()==profile and profile.parent==root
assert profile.is_dir() and profile.stat().st_uid==os.getuid()
for path in Path('/proc').iterdir():
    if not path.name.isdecimal():continue
    try:
        if path.stat().st_uid!=os.getuid():continue
        command=(path/'cmdline').read_bytes().split(b'\0')
        assert os.fsencode(profile) not in command,'Owned Firefox profile is still in use'
    except (FileNotFoundError,ProcessLookupError,PermissionError):pass
prefs={'media.autoplay.default':0,'media.autoplay.blocking_policy':0,
       'media.autoplay.block-webaudio':False,'browser.shell.checkDefaultBrowser':False,
       'browser.aboutwelcome.enabled':False,'browser.startup.homepage_override.mstone':'ignore',
       'browser.cache.disk.enable':False,'toolkit.telemetry.enabled':False,
       'datareporting.healthreport.uploadEnabled':False}
expected=''.join('user_pref('+json.dumps(key)+','+json.dumps(value)+');\n' for key,value in prefs.items())
assert (profile/'user.js').read_text()==expected,'Unexpected profile preferences; preserve for review'
size=sum(path.stat().st_size for path in profile.rglob('*') if path.is_file())
shutil.rmtree(profile)
print(json.dumps({'scope':'only the disposable profile created for this project test',
                  'profile_removed':not profile.exists(),'removed_bytes':size,
                  'generated_music_preserved':(root/'tone.wav').exists()},indent=2))
