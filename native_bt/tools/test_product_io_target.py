import json,pathlib,re,subprocess
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent
commands=json.loads((root/'_build/esp32/compile_commands.json').read_text());e=next(x for x in commands if x['file'].endswith('probe_audio.c'))
base=re.sub(r' -o .*? -c .*$', '',e['command']);compiler,flags=base.split(' ',1)
flags=re.sub(r' -DDBT_PROBE_PAIRING_PEER_BYTES=[^ ]+', '', ' '+flags)
for part in ('esp-idf/components/esp_driver_spi/include','esp-idf/components/driver/i2c/include','esp-idf/components/esp_driver_gpio/include','esp-idf/components/spiffs/include'):
 flags+=' -I"'+(ws/part).as_posix()+'"'
flags+=' -DDBT_PROBE_SOUND_BUILD=1'
rsp=root/'_build/led/product_io.rsp';rsp.parent.mkdir(exist_ok=True);rsp.write_text(flags)
subprocess.run([compiler,'@'+str(rsp),'-fsyntax-only',str(root/'esp32_probe/main/probe_product_io.c')],cwd=e['directory'],check=True)
print('PASS Xtensa product IO syntax')
