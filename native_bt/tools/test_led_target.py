import json, pathlib, re, subprocess
root=pathlib.Path(__file__).resolve().parents[1]; ws=root.parent
commands=json.loads((root/'_build/esp32/compile_commands.json').read_text())
entry=next(x for x in commands if x['file'].endswith('probe_audio.c'))
base=re.sub(r' -o .*? -c .*$', '', entry['command'])
compiler,flags=base.split(' ',1)
flags=re.sub(r' -DDBT_PROBE_PAIRING_PEER_BYTES=[^ ]+', '', ' '+flags)
flags += ' -I"'+(ws/'esp-idf/components/esp_driver_spi/include').as_posix()+'"'
flags += ' -I"'+(ws/'esp-idf/components/driver/i2c/include').as_posix()+'"'
flags += ' -I"'+(ws/'esp-idf/components/esp_driver_gpio/include').as_posix()+'"'
rsp=root/'_build/led/target.rsp';rsp.parent.mkdir(exist_ok=True);rsp.write_text(flags)
for name in ('dbt_led.c','esp32/dbt_led_esp32.c','dbt_input.c','esp32/dbt_input_esp32.c'):
    print('checking',name,flush=True)
    subprocess.run([compiler,'@'+str(rsp),'-fsyntax-only',str(root/'src'/name)],cwd=entry['directory'],check=True)
