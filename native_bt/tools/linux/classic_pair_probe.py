"""Pair only with the identified diagnostic ESP32; never print link keys/PINs."""
import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
import json,time

ADDRESS='6C:C8:40:42:BF:66'
DEVICE='/org/bluez/hci0/dev_'+ADDRESS.replace(':','_')
AGENT='/native_bt/pairing_agent'
DBusGMainLoop(set_as_default=True)
bus=dbus.SystemBus();context=GLib.MainContext.default()
manager=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')
adapter=dbus.Interface(bus.get_object('org.bluez','/org/bluez/hci0'),'org.bluez.Adapter1')
agents=dbus.Interface(bus.get_object('org.bluez','/org/bluez'),'org.bluez.AgentManager1')
def objects():return manager.GetManagedObjects(timeout=5)
def dispatch(milliseconds):
    until=time.monotonic()+milliseconds/1000
    while True:
        while context.pending():context.iteration(False)
        if time.monotonic()>=until:return
        time.sleep(max(0,min(.005,until-time.monotonic())))
def check_device(device):
    if str(device)!=DEVICE:raise dbus.exceptions.DBusException('Unrelated device',name='org.bluez.Error.Rejected')
class Agent(dbus.service.Object):
    @dbus.service.method('org.bluez.Agent1',in_signature='',out_signature='')
    def Release(self):pass
    @dbus.service.method('org.bluez.Agent1',in_signature='o',out_signature='s')
    def RequestPinCode(self,device):check_device(device);raise dbus.exceptions.DBusException('Legacy PIN not enabled in this probe',name='org.bluez.Error.Rejected')
    @dbus.service.method('org.bluez.Agent1',in_signature='o',out_signature='u')
    def RequestPasskey(self,device):check_device(device);raise dbus.exceptions.DBusException('No input capability',name='org.bluez.Error.Rejected')
    @dbus.service.method('org.bluez.Agent1',in_signature='ouq',out_signature='')
    def DisplayPasskey(self,device,passkey,entered):check_device(device)
    @dbus.service.method('org.bluez.Agent1',in_signature='os',out_signature='')
    def DisplayPinCode(self,device,pincode):check_device(device)
    @dbus.service.method('org.bluez.Agent1',in_signature='ou',out_signature='')
    def RequestConfirmation(self,device,passkey):check_device(device)
    @dbus.service.method('org.bluez.Agent1',in_signature='o',out_signature='')
    def RequestAuthorization(self,device):check_device(device)
    @dbus.service.method('org.bluez.Agent1',in_signature='os',out_signature='')
    def AuthorizeService(self,device,uuid):check_device(device)
    @dbus.service.method('org.bluez.Agent1',in_signature='',out_signature='')
    def Cancel(self):pass

def main():
    report={'scope':'owned Classic pairing/bond diagnostics; not A2DP/audio qualification','passed':False}
    scanned=registered=attempted=False;device=None;preferred=None
    try:
        before=objects();local=before['/org/bluez/hci0']['org.bluez.Adapter1']
        report['adapter_address']=str(local['Address'])
        if report['adapter_address']!='E8:47:3A:EC:51:9A':raise RuntimeError('The source adapter differs from the compiled peer policy')
        if any(v.get('org.bluez.Device1',{}).get('Connected') for v in before.values()):raise RuntimeError('Preserve active Bluetooth sessions')
        adapter.SetDiscoveryFilter(dbus.Dictionary({'Transport':'auto','Pattern':ADDRESS},signature='sv'),timeout=5)
        adapter.StartDiscovery(timeout=5);scanned=True;deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            dispatch(100);properties=objects().get(DEVICE,{}).get('org.bluez.Device1',{})
            if properties.get('Name')=='NATIVE_BT_PROBE':break
        else:raise RuntimeError('The named diagnostic peripheral was not discovered')
        report['already_known']=DEVICE in before;report['initial_paired']=bool(properties.get('Paired'))
        if properties.get('Connected'):raise RuntimeError('Target already connected')
        br=objects().get(DEVICE,{}).get('org.bluez.Bearer.BREDR1',{})
        if br.get('Paired') and br.get('Bonded'):
            # Calling generic Pair again can select the still-unpaired LE bearer.
            # Reuse a durable BR bond; this is not a new SSP/encryption result.
            report.update(passed=True,fresh_pairing=False,paired=True,bonded=True,bredr_paired=True,
                          elapsed_seconds=0,disconnected=True,pair_method={'existing_classic_bond':True})
        else:
            report['fresh_pairing']=True
            agent=Agent(bus,AGENT);agents.RegisterAgent(AGENT,'NoInputNoOutput',timeout=5);registered=True
            target=bus.get_object('org.bluez',DEVICE);device=dbus.Interface(target,'org.bluez.Device1')
            if 'PreferredBearer' in properties:
                preferred=str(properties['PreferredBearer']);dbus.Interface(target,'org.freedesktop.DBus.Properties').Set('org.bluez.Device1','PreferredBearer','bredr',timeout=5)
            outcome={}
            def success():outcome['success']=True
            def error(problem):outcome['error']=str(problem)
            began=time.monotonic();attempted=True
            device.Pair(reply_handler=success,error_handler=error,timeout=55)
            states=[]
            while not outcome and time.monotonic()-began<58:
                dispatch(100);current=objects().get(DEVICE,{})
                p=current.get('org.bluez.Device1',{});br=current.get('org.bluez.Bearer.BREDR1',{});le=current.get('org.bluez.Bearer.LE1',{})
                state=[bool(p.get('Paired')),bool(p.get('Bonded')),bool(br.get('Connected')),bool(le.get('Connected')),bool(p.get('ServicesResolved'))]
                if not states or states[-1]['state']!=state:states.append({'seconds':round(time.monotonic()-began,3),'state':state})
            report['pair_method']=outcome;report['states_paired_bonded_bredr_le_services']=states
            report['elapsed_seconds']=round(time.monotonic()-began,3)
            current=objects().get(DEVICE,{});p=current.get('org.bluez.Device1',{});br=current.get('org.bluez.Bearer.BREDR1',{})
            report['paired']=bool(p.get('Paired'));report['bonded']=bool(p.get('Bonded'));report['bredr_paired']=bool(br.get('Paired'))
            report['passed']=bool(outcome.get('success')) and report['paired'] and report['bonded'] and report['bredr_paired']
            dispatch(1000)
    except Exception as error:report['error']=str(error)
    finally:
        if attempted and device:
            try:device.Disconnect(timeout=8);report['disconnected']=True
            except Exception as error:report['disconnect_error']=str(error)
        if preferred is not None:
            try:dbus.Interface(bus.get_object('org.bluez',DEVICE),'org.freedesktop.DBus.Properties').Set('org.bluez.Device1','PreferredBearer',preferred,timeout=5)
            except Exception as error:report['preferred_restore_error']=str(error)
        if registered:
            try:agents.UnregisterAgent(AGENT,timeout=5)
            except Exception as error:report['agent_cleanup_error']=str(error)
        if scanned:
            try:adapter.StopDiscovery(timeout=5)
            except Exception as error:report['scan_cleanup_error']=str(error)
        if report.get('passed'):
            dispatch(1500)
            retained=objects().get(DEVICE,{}).get('org.bluez.Bearer.BREDR1',{})
            report['bond_retained_after_cleanup']=bool(retained.get('Paired')) and bool(retained.get('Bonded'))
            report['passed']=report['bond_retained_after_cleanup']
        print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1

raise SystemExit(main())
