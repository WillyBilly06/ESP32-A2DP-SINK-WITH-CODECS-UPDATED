"""Read only the signed-in user's audio/browser routing state; no media changes."""
import json,os,subprocess
report={'scope':'read-only user audio/browser session inventory'}
uid=os.getuid();os.environ['XDG_RUNTIME_DIR']='/run/user/'+str(uid)
os.environ['DBUS_SESSION_BUS_ADDRESS']='unix:path=/run/user/'+str(uid)+'/bus'
try:
    import dbus
    session=dbus.SessionBus();players=[]
    for name in session.list_names():
        if str(name).startswith('org.mpris.MediaPlayer2.'):
            properties=dbus.Interface(session.get_object(str(name),'/org/mpris/MediaPlayer2'),'org.freedesktop.DBus.Properties')
            players.append({'name':str(name),'state':str(properties.Get('org.mpris.MediaPlayer2.Player','PlaybackStatus'))})
    report['players']=players
except Exception as error:report['session_error']=str(error)
try:
    report['default_sink']=subprocess.check_output(['pactl','get-default-sink'],text=True,timeout=5).strip()
    for kind in ['sinks','sink-inputs']:
        data=json.loads(subprocess.check_output(['pactl','-f','json','list',kind],text=True,timeout=5))
        report[kind]=[{'index':row['index'],'name':row.get('name'),'sink':row.get('sink'),
            'application':row.get('properties',{}).get('application.name')} for row in data]
except Exception as error:report['pulse_error']=str(error)
names=subprocess.check_output(['ps','-u',str(uid),'-o','comm='],text=True,timeout=5).splitlines()
report['firefox_processes']=sum('firefox' in name.lower() for name in names)
print(json.dumps(report,indent=2))
