package com.bdkaudio.idf61check;

import android.app.Instrumentation;
import android.bluetooth.*;
import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.media.MediaPlayer;
import android.media.AudioAttributes;
import android.media.AudioManager;
import android.media.AudioDeviceInfo;
import android.companion.CompanionDeviceManager;
import android.companion.AssociationRequest;
import android.companion.AssociationInfo;
import android.companion.BluetoothDeviceFilter;
import android.content.IntentSender;
import android.content.Intent;
import java.nio.file.Files;
import java.util.*;
import java.util.concurrent.*;
import org.json.*;

/** Finite, explicitly selected speaker checks using normal Android BLE APIs.
 * Sound tests replace/delete speaker files: retain a SPIFFS backup first.
 * OTA mode writes the supplied image and reboots the selected speaker.
 * This checks protocol state; it cannot establish acoustic/physical output.
 */
public final class FeatureCheck extends Instrumentation {
    private static final UUID SERVICE = UUID.fromString("12345678-1234-1234-1234-123456789000");
    private static final UUID CMD = UUID.fromString("12345678-1234-1234-1234-123456789001");
    private static final UUID STATUS = UUID.fromString("12345678-1234-1234-1234-123456789002");
    private static final UUID METER = UUID.fromString("12345678-1234-1234-1234-123456789003");
    private static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private final BlockingQueue<String> callbacks = new LinkedBlockingQueue<>();
    private final BlockingQueue<byte[]> statuses = new LinkedBlockingQueue<>();
    private final JSONArray results = new JSONArray();
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic command;
    private volatile int mtu = 23, meters = 0, nonzeroMeters = 0;
    private Bundle args;
    private MediaPlayer continuous;
    private AudioManager continuousAudio;
    private int continuousVolume;

    @Override public void onCreate(Bundle arguments) { super.onCreate(arguments); args = arguments; start(); }
    private final BluetoothGattCallback callback = new BluetoothGattCallback() {
        @Override public void onConnectionStateChange(BluetoothGatt g, int s, int state) {
            if (state == BluetoothProfile.STATE_CONNECTED) callbacks.add("connect:"+s);
            else if (state == BluetoothProfile.STATE_DISCONNECTED) Log.i("BDK_IDF61", "GATT disconnected status=" + s);
        }
        @Override public void onMtuChanged(BluetoothGatt g, int value, int s) { mtu=value; callbacks.add("mtu:"+s); }
        @Override public void onServicesDiscovered(BluetoothGatt g, int s) { callbacks.add("services:"+s); }
        @Override public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int s) { callbacks.add("descriptor:"+s); }
        @Override public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int s) { callbacks.add("write:"+s); }
        @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c, byte[] data) {
            if (c.getUuid().equals(STATUS)) statuses.add(data.clone());
            else if (c.getUuid().equals(METER) && data.length == 3) {
                meters++;
                if(data[0]!=0 || data[1]!=0 || data[2]!=0)nonzeroMeters++;
            }
        }
    };
    private void check(boolean ok, String message) { if (!ok) throw new AssertionError(message); }
    private void callbackOk(String kind) throws Exception {
        long until=System.nanoTime()+TimeUnit.SECONDS.toNanos(25);
        while(System.nanoTime()<until){
            String event=callbacks.poll(1,TimeUnit.SECONDS);
            if(event==null || !event.startsWith(kind+":"))continue;
            check(event.equals(kind+":"+BluetoothGatt.GATT_SUCCESS),"GATT callback: "+event);return;
        }
        throw new AssertionError("GATT callback timeout: "+kind);
    }
    private byte[] waitStatus(int id) throws Exception {
        long until=System.nanoTime()+TimeUnit.SECONDS.toNanos(25);
        while (System.nanoTime()<until) {
            byte[] b=statuses.poll(1,TimeUnit.SECONDS);
            if(b==null) continue;
            if((b[0]&255)==0x11 || (b[0]&255)==0x23 || (b[0]&255)==0x33)
                throw new AssertionError("Peripheral error " + hex(b));
            if((b[0]&255)==id) return b;
        }
        throw new AssertionError("Notification timeout 0x" + Integer.toHexString(id));
    }
    private void write(byte[] packet) throws Exception {
        callbacks.clear();
        check(gatt.writeCharacteristic(command,packet,BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)==BluetoothStatusCodes.SUCCESS,"write rejected");
        callbackOk("write");
    }
    private void send(int... values) throws Exception {
        byte[] b=new byte[values.length]; for(int i=0;i<b.length;i++)b[i]=(byte)values[i]; write(b);
    }
    private byte[] sync() throws Exception { statuses.clear(); send(0xf0); return waitStatus(0xf0); }
    private void result(String name, String detail) throws Exception {
        JSONObject row=new JSONObject();row.put("feature",name);row.put("result","pass");row.put("detail",detail);
        results.put(row);Bundle b=new Bundle();b.putString("feature",row.toString());sendStatus(0,b);
    }
    private static String hex(byte[] b) { StringBuilder s=new StringBuilder();for(byte v:b)s.append(String.format("%02x",v&255));return s.toString(); }
    private void eq(int b,int m,int t) throws Exception {
        send(1,b,m,t);waitStatus(0x10);byte[] s=sync();
        check(s[1]==(byte)b && s[2]==(byte)m && s[3]==(byte)t,"EQ readback "+hex(s));
    }
    private void control(int flags) throws Exception {
        send(3,flags);waitStatus(0x10);check((sync()[4]&255)==flags,"Control readback");
    }
    private void led(byte[] payload) throws Exception {
        byte[] p=new byte[11];p[0]=5;System.arraycopy(payload,0,p,1,10);write(p);waitStatus(0x10);
        byte[] s=sync();for(int i=0;i<10;i++)check(s[5+i]==payload[i],"LED field "+i+" "+hex(s));
    }
    private void rename(byte[] name) throws Exception {
        byte[] p=new byte[name.length+2];p[0]=4;System.arraycopy(name,0,p,1,name.length);write(p);waitStatus(0x10);
        byte[] s=sync();int n=s[16]&255;check(n==name.length,"Name length");
        for(int i=0;i<n;i++)check(s[17+i]==name[i],"Name readback");
    }
    private void controls() throws Exception {
        byte[] initial=sync();check(initial.length>=18,"Full status length");result("full_status",hex(initial));
        statuses.clear();send(0xff);waitStatus(0xff);result("ping","PONG received");
        int n=initial[16]&255;byte[] name=Arrays.copyOfRange(initial,17,17+n);
        byte[] lights=Arrays.copyOfRange(initial,5,15);
        try {
            for(int band=0;band<3;band++) for(int value:new int[]{-12,0,12}) {
                int[] e={0,0,0};e[band]=value;eq(e[0],e[1],e[2]);
            }
            result("equalizer","All three bands at -12, 0, +12 with status readback");
            for(int flags=0;flags<8;flags++)control(flags);
            result("sound_options","All 8 bass boost/output swap/full-range combinations");
            for(int effect=0;effect<22;effect++) {
                send(6,effect);waitStatus(0x10);check((sync()[5]&255)==effect,"Effect "+effect);
            }
            result("led_effects","All 22 user effects selected and read back");
            for(int gradient=0;gradient<5;gradient++) led(new byte[]{21,40,60,(byte)255,0,0,0,0,(byte)255,(byte)gradient});
            result("led_colors_speed_gradients","Ambient colors, speed and all 5 gradient modes read back");
            for(int bright:new int[]{0,1,37,100}) {
                send(7,bright);waitStatus(0x10);check(Math.abs((sync()[6]&255)-bright)<=1,"Brightness "+bright);
            }
            result("led_brightness_off","0, 1, 37, 100 including Off; conversion tolerance 1");
            rename("BDK IDF61 Test".getBytes("UTF-8"));result("rename","Classic/BLE setting status readback");
            send(0x10,1);waitStatus(0x10);check((sync()[15]&0x80)!=0,"Mute readback");
            send(0x10,0);waitStatus(0x10);check((sync()[15]&0x80)==0,"Unmute readback");
            result("notification_mute","Mute and unmute status readback");
        } finally {
            eq(initial[1],initial[2],initial[3]);control(initial[4]&255);led(lights);rename(name);
            send(0x10,(initial[15]&0x80)!=0?1:0);waitStatus(0x10);
        }
        result("restore_settings","Original EQ/control/LED/name/mute restored");
    }
    private byte[] fixture(String file) throws Exception { return Files.readAllBytes(new java.io.File(getTargetContext().getFilesDir(),file).toPath()); }
    private void transfer(int commandId,byte[] bytes,int ready) throws Exception {
        int at=0,seq=0,capacity=Math.min(mtu-5,500);
        while(at<bytes.length) {
            int count=Math.min(capacity,bytes.length-at);
            if(bytes.length-at-count==1)count--;
            byte[] p=new byte[count+2];p[0]=(byte)commandId;p[1]=(byte)seq++;
            System.arraycopy(bytes,at,p,2,count);statuses.clear();write(p);if(ready>=0)waitStatus(ready);at+=count;
            if((seq%100)==0)Log.i("BDK_IDF61","Transfer bytes="+at+"/"+bytes.length);
        }
    }
    private void sounds() throws Exception {
        byte[] wav=fixture("sound.wav");
        for(int type=0;type<4;type++) {
            statuses.clear();send(0x12,type,wav.length&255,(wav.length>>8)&255,(wav.length>>16)&255);waitStatus(0x31);
            transfer(0x13,wav,0x31);statuses.clear();send(0x14);waitStatus(0x32);
            check((sync()[15]&(1<<type))!=0,"Uploaded sound presence "+type);
            result("sound_upload_"+type,"WAV transfer and completion/presence verified");
            statuses.clear();send(0x11,type);waitStatus(0x10);
            boolean deleted=false;
            for(int i=0;i<30;i++){Thread.sleep(100);if((sync()[15]&(1<<type))==0){deleted=true;break;}}
            check(deleted,"Deleted sound presence "+type);
            result("sound_delete_"+type,"Deleted test sound and checked status");
        }
    }
    private void audio() throws Exception {
        byte[] initial=sync();
        AudioManager audio=(AudioManager)getTargetContext().getSystemService(Context.AUDIO_SERVICE);
        int originalVolume=audio.getStreamVolume(AudioManager.STREAM_MUSIC);
        MediaPlayer player=new MediaPlayer();
        try {
            player.setAudioAttributes(new AudioAttributes.Builder().setUsage(AudioAttributes.USAGE_MEDIA).setContentType(AudioAttributes.CONTENT_TYPE_MUSIC).build());
            player.setDataSource(new java.io.File(getTargetContext().getFilesDir(),"playback.wav").getPath());
            player.prepare();player.setLooping(true);
            audio.setStreamVolume(AudioManager.STREAM_MUSIC,Math.min(originalVolume,2),0);
            int before=meters;player.start();Thread.sleep(1500);
            AudioDeviceInfo route=player.getRoutedDevice();
            check(route!=null && route.getType()==AudioDeviceInfo.TYPE_BLUETOOTH_A2DP &&
                  route.getAddress().equalsIgnoreCase(args.getString("address")),"Playback not routed to selected A2DP speaker");
            result("audio_route","A2DP route="+route.getProductName()+", address="+route.getAddress());
            Thread.sleep(10000);
            check(meters-before>5,"No sustained meter notifications during playback");
            result("audio_and_ble_meters",Integer.toString(meters-before)+" meter notifications during playback");
            eq(3,-3,3);eq(0,0,0);control(1);control(0);
            result("audio_control_coexistence","EQ and bass boost applied/read back while A2DP played");
        } finally {
            player.release();audio.setStreamVolume(AudioManager.STREAM_MUSIC,originalVolume,0);
            eq(initial[1],initial[2],initial[3]);control(initial[4]&255);
        }
    }
    private Object get(Object object,String method) throws Exception { return object.getClass().getMethod(method).invoke(object); }
    private Object copyPreference(Object source,int priority) throws Exception {
        Class<?> c=source.getClass();
        return c.getConstructor(int.class,int.class,int.class,int.class,int.class,long.class,long.class,long.class,long.class).newInstance(
            ((Number)get(source,"getCodecType")).intValue(),priority,
            ((Number)get(source,"getSampleRate")).intValue(),((Number)get(source,"getBitsPerSample")).intValue(),
            ((Number)get(source,"getChannelMode")).intValue(),((Number)get(source,"getCodecSpecific1")).longValue(),
            ((Number)get(source,"getCodecSpecific2")).longValue(),((Number)get(source,"getCodecSpecific3")).longValue(),
            ((Number)get(source,"getCodecSpecific4")).longValue());
    }
    private void startContinuous() throws Exception {
        continuousAudio=(AudioManager)getTargetContext().getSystemService(Context.AUDIO_SERVICE);
        continuousVolume=continuousAudio.getStreamVolume(AudioManager.STREAM_MUSIC);
        continuous=new MediaPlayer();
        continuous.setAudioAttributes(new AudioAttributes.Builder().setUsage(AudioAttributes.USAGE_MEDIA).setContentType(AudioAttributes.CONTENT_TYPE_MUSIC).build());
        continuous.setDataSource(new java.io.File(getTargetContext().getFilesDir(),"playback.wav").getPath());
        continuous.prepare();continuous.setLooping(true);continuousAudio.setStreamVolume(AudioManager.STREAM_MUSIC,4,0);continuous.start();
        Thread.sleep(1500);AudioDeviceInfo route=continuous.getRoutedDevice();
        check(route!=null && route.getType()==AudioDeviceInfo.TYPE_BLUETOOTH_A2DP && route.getAddress().equalsIgnoreCase(args.getString("address")),"Continuous audio not on selected speaker");
        result("continuous_audio","Started quiet looping test tone on selected A2DP speaker");
    }
    private void stopContinuous() {
        if(continuous!=null){continuous.release();continuous=null;}
        if(continuousAudio!=null){continuousAudio.setStreamVolume(AudioManager.STREAM_MUSIC,continuousVolume,0);continuousAudio=null;}
    }
    private void associate(String address) throws Exception {
        CompanionDeviceManager manager=getTargetContext().getSystemService(CompanionDeviceManager.class);
        for(String known:manager.getAssociations())if(known.equalsIgnoreCase(address)){result("association","Existing association for selected speaker");return;}
        CountDownLatch ready=new CountDownLatch(1);final String[] error={null};
        AssociationRequest request=new AssociationRequest.Builder().setSingleDevice(true)
            .addDeviceFilter(new BluetoothDeviceFilter.Builder().setAddress(address).build()).build();
        manager.associate(request,getTargetContext().getMainExecutor(),new CompanionDeviceManager.Callback(){
            @Override public void onAssociationPending(IntentSender sender){
                try {getTargetContext().startIntentSender(sender,new Intent().addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),Intent.FLAG_ACTIVITY_NEW_TASK,0,0);}
                catch(Exception e){error[0]=e.toString();ready.countDown();}
            }
            @Override public void onAssociationCreated(AssociationInfo info){ready.countDown();}
            @Override public void onFailure(CharSequence reason){error[0]=String.valueOf(reason);ready.countDown();}
        });
        check(ready.await(60,TimeUnit.SECONDS) && error[0]==null,"Companion association: "+error[0]);
        result("association","Normal Android companion consent completed for selected speaker");
    }
    private void codecs(boolean mutate) throws Exception {
        BluetoothAdapter adapter=((BluetoothManager)getTargetContext().getSystemService(Context.BLUETOOTH_SERVICE)).getAdapter();
        final BluetoothA2dp[] profile=new BluetoothA2dp[1];CountDownLatch ready=new CountDownLatch(1);
        check(adapter.getProfileProxy(getTargetContext(),new BluetoothProfile.ServiceListener(){
            public void onServiceConnected(int id,BluetoothProfile proxy){if(id==BluetoothProfile.A2DP){profile[0]=(BluetoothA2dp)proxy;ready.countDown();}}
            public void onServiceDisconnected(int id){}
        },BluetoothProfile.A2DP),"A2DP profile request");
        Object original=null;BluetoothDevice device=adapter.getRemoteDevice(args.getString("address"));
        try {
            check(ready.await(8,TimeUnit.SECONDS),"A2DP proxy timeout");
            check(profile[0].getConnectionState(device)==BluetoothProfile.STATE_CONNECTED,"Selected speaker has no A2DP connection");
            java.lang.reflect.Method statusMethod=profile[0].getClass().getMethod("getCodecStatus",BluetoothDevice.class);
            Object status=statusMethod.invoke(profile[0],device);original=get(status,"getCodecConfig");
            result("codec_before",original.toString());
            Object caps=get(status,"getCodecsSelectableCapabilities");List<Object> selectable=new ArrayList<>();
            if(caps instanceof Iterable<?>)for(Object c:(Iterable<?>)caps)selectable.add(c);
            else if(caps instanceof Object[])selectable.addAll(Arrays.asList((Object[])caps));
            for(Object cap:selectable)result("selectable_codec",cap.toString());
            if(args.getString("mode", "").equals("restore_ldac")){
                boolean available=false;for(Object cap:selectable)if(((Number)get(cap,"getCodecType")).intValue()==4)available=true;
                check(available,"LDAC not selectable");
                Class<?> config=Class.forName("android.bluetooth.BluetoothCodecConfig");
                Object desired=config.getConstructor(int.class,int.class,int.class,int.class,int.class,long.class,long.class,long.class,long.class)
                    .newInstance(4,1000000,8,4,2,0L,0L,0L,0L);
                profile[0].getClass().getMethod("setCodecConfigPreference",BluetoothDevice.class,config).invoke(profile[0],device,desired);
                Object active=null;boolean restored=false;
                for(int i=0;i<80;i++){
                    Thread.sleep(100);active=get(statusMethod.invoke(profile[0],device),"getCodecConfig");
                    if(((Number)get(active,"getCodecType")).intValue()==4 && ((Number)get(active,"getSampleRate")).intValue()==8 && ((Number)get(active,"getBitsPerSample")).intValue()==4){restored=true;break;}
                }
                check(restored,"LDAC 96 kHz/32-bit restoration failed");result("original_ldac_restored",active.toString());return;
            }
            if(!mutate)return;
            Class<?> config=Class.forName("android.bluetooth.BluetoothCodecConfig");
            java.lang.reflect.Constructor<?> ctor=config.getConstructor(int.class,int.class,int.class,int.class,int.class,long.class,long.class,long.class,long.class);
            java.lang.reflect.Method setter=profile[0].getClass().getMethod("setCodecConfigPreference",BluetoothDevice.class,config);
            startContinuous();
            for(Object cap:selectable){
                int type=((Number)get(cap,"getCodecType")).intValue();
                int rates=((Number)get(cap,"getSampleRate")).intValue();
                int depths=((Number)get(cap,"getBitsPerSample")).intValue();
                int channels=((Number)get(cap,"getChannelMode")).intValue();
                int rate=Integer.lowestOneBit(rates),depth=Integer.lowestOneBit(depths),channel=Integer.highestOneBit(channels);
                Object desired=ctor.newInstance(type,1000000,rate,depth,channel,0L,0L,0L,0L);
                check(profile[0].getConnectionState(device)==BluetoothProfile.STATE_CONNECTED,"Speaker disconnected before codec request");
                setter.invoke(profile[0],device,desired);
                Object active=null;boolean matched=false;
                for(int i=0;i<100;i++){
                    Thread.sleep(100);active=get(statusMethod.invoke(profile[0],device),"getCodecConfig");
                    if(((Number)get(active,"getCodecType")).intValue()==type){matched=true;break;}
                }
                check(matched,"Codec switch did not match type "+type);result("codec_selected",active.toString());
                int before=meters,nonzero=nonzeroMeters;Thread.sleep(6000);
                check(meters-before>5,"No BLE meters after live codec switch type "+type);
                check(nonzeroMeters-nonzero>5,"No nonzero audio levels after live codec switch type "+type);
                byte[] initial=sync();eq(3,-3,3);control(1);eq(initial[1],initial[2],initial[3]);control(initial[4]&255);
                result("live_codec_audio_"+type,(meters-before)+" meter packets, "+(nonzeroMeters-nonzero)+" nonzero; EQ/control readback during continuous playback");
            }
        } finally {
            stopContinuous();
            if(profile[0]!=null){
                if(mutate && original!=null && profile[0].getConnectionState(device)==BluetoothProfile.STATE_CONNECTED){
                    profile[0].getClass().getMethod("setCodecConfigPreference",BluetoothDevice.class,original.getClass()).invoke(profile[0],device,copyPreference(original,1000000));
                    Object restored=null;
                    for(int i=0;i<80;i++){
                        Thread.sleep(100);restored=get(profile[0].getClass().getMethod("getCodecStatus",BluetoothDevice.class).invoke(profile[0],device),"getCodecConfig");
                        if(restored.toString().equals(original.toString()))break;
                    }
                    check(restored.toString().equals(original.toString()),"Original codec tuple not restored: "+restored);
                    result("codec_restored",restored.toString());
                }
                adapter.closeProfileProxy(BluetoothProfile.A2DP,profile[0]);
            }
        }
    }
    private void ota() throws Exception {
        byte[] image=fixture("firmware.bin");
        statuses.clear();send(0x20,image.length&255,(image.length>>8)&255,(image.length>>16)&255,(image.length>>24)&255);waitStatus(0x21);
        statuses.clear();send(0x23);waitStatus(0x10);result("ota_abort","BEGIN/READY/ABORT acknowledgement");
        statuses.clear();send(0x20,image.length&255,(image.length>>8)&255,(image.length>>16)&255,(image.length>>24)&255);waitStatus(0x21);
        transfer(0x21,image,-1);statuses.clear();send(0x22);waitStatus(0x22);
        result("ota_transfer","Raw image with serialized GATT writes, COMPLETE received; verify reboot separately");
    }
    @Override public void onStart() {
        Bundle finish=new Bundle();
        try {
            String address=args.getString("address");check(address!=null,"Explicit speaker address required");
            BluetoothManager manager=(BluetoothManager)getTargetContext().getSystemService(Context.BLUETOOTH_SERVICE);
            BluetoothDevice device=manager.getAdapter().getRemoteDevice(address);
            if(args.getString("mode", "").equals("associate")){
                associate(address);finish.putString("result","PASS");return;
            }
            if (args.getString("mode", "").equals("identity")) {
                result("identity", "name="+device.getName()+", bondState="+device.getBondState());
                finish.putString("result", "PASS");
                return;
            }
            gatt=device.connectGatt(getTargetContext(),false,callback,BluetoothDevice.TRANSPORT_LE);callbackOk("connect");
            check(gatt.requestMtu(517),"MTU request");callbackOk("mtu");
            check(gatt.discoverServices(),"Service discovery");callbackOk("services");
            BluetoothGattService service=gatt.getService(SERVICE);
            StringBuilder serviceIds=new StringBuilder();for(BluetoothGattService found:gatt.getServices())serviceIds.append(found.getUuid()).append(" ");
            check(service!=null,"BDK service missing; discovered="+serviceIds);
            command=service.getCharacteristic(CMD);check(command!=null,"Command characteristic missing");
            for(UUID id:new UUID[]{STATUS,METER}) {
                BluetoothGattCharacteristic c=service.getCharacteristic(id);check(c!=null,"Characteristic "+id);
                check(gatt.setCharacteristicNotification(c,true),"Subscribe "+id);
                check(gatt.writeDescriptor(c.getDescriptor(CCCD),BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)==BluetoothStatusCodes.SUCCESS,"CCCD write");callbackOk("descriptor");
            }
            result("discovery_subscriptions","Service, command, status and meters; MTU "+mtu);
            String mode=args.getString("mode","controls");
            if(mode.equals("controls"))controls();
            else if(mode.equals("sounds"))sounds();
            else if(mode.equals("live_sounds")){
                try{startContinuous();sounds();result("sound_audio_coexistence",meters+" meter packets while sound slots transferred/deleted");}
                finally{stopContinuous();}
            }
            else if(mode.equals("ota"))ota();
            else if(mode.equals("audio"))audio();
            else if(mode.equals("codec_probe"))codecs(false);
            else if(mode.equals("restore_ldac"))codecs(false);
            else if(mode.equals("codecs"))codecs(true);
            else if(mode.equals("status"))result("status",hex(sync()));
            else throw new AssertionError("Unknown mode "+mode);
            result("meter_packets",Integer.toString(meters)+" observed; zero without playback is not a failure");
            finish.putString("result","PASS");
        } catch(Throwable e) {
            finish.putString("result","FAIL");finish.putString("failure",e.toString());Log.e("BDK_IDF61","Feature check failed",e);
            if(e.getCause()!=null)finish.putString("cause",e.getCause().toString());
        } finally {
            if(gatt!=null){gatt.disconnect();gatt.close();}
            finish.putString("checks",results.toString());finish(0,finish);
        }
    }
}
