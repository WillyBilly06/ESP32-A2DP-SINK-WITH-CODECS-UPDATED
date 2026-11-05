'use strict';
const TOKEN='__CONTROL_TOKEN__';
const $=id=>document.getElementById(id);
const stages=['All stages','System','HCI / transport','Profiles','Decoder / queue','DSP / render','I2S / DMA','BLE / controls','Memory / scheduler','OTA / storage','Product I/O'];
const icons=['◫','◇','⇄','⌘','▥','∿','≈','◉','▤','↥','⌁'];
const codecNames={0:'Waiting',1:'SBC',2:'Opus Android',3:'Opus05',4:'AAC-LC',5:'aptX',6:'aptX HD',7:'aptX LL',8:'LDAC',9:'LHDC V5'};
let events=[],cursor=0,state=null,stage='All stages',paused=false,selected=null,clearEpoch=null,lastRender=0,renderDirty=true;
const fmt=n=>Number(n||0).toLocaleString();
function element(tag,text,cls){const node=document.createElement(tag);if(text!==undefined)node.textContent=text;if(cls)node.className=cls;return node;}
function toast(text){$('toast').textContent=text;$('toast').style.opacity='1';setTimeout(()=>$('toast').style.opacity='0',3000);}
async function command(action,extra={}){
  try{const r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json','X-Monitor-Token':TOKEN},body:JSON.stringify({action,...extra})});if(!r.ok)throw Error(await r.text());
    const receipt=await r.json();toast(action==='release'?'Releasing COM6…':action==='resume'?'Connecting to UART…':action==='clear'?'Buffer cleared':'Capture requested');
    if(action==='release'||action==='resume'){
      const end=Date.now()+5000;while(Date.now()<end){const s=await(await fetch('/state',{cache:'no-store'})).json();update(s);
        if(s.applied_command_id===receipt.id&&s.reader_mode===(action==='release'?'released':'listening')&&s.connected===(action==='resume')){toast(action==='release'?'COM6 is released and available for flashing':'Connected to verified COM6');return;}
        await new Promise(resolve=>setTimeout(resolve,120));}
      toast('Reader state is not yet acknowledged — check the status above');
    }
  }catch(e){toast('Command failed: '+e.message);}
}
for(let i=0;i<stages.length;i++){
  const button=element('button',undefined,i===0?'active':'');button.dataset.stage=stages[i];button.title=stages[i];
  button.append(element('span',icons[i],'nav-icon'),element('span',stages[i]),element('span','0','count'));
  button.addEventListener('click',()=>{stage=stages[i];document.querySelectorAll('#stages button').forEach(x=>x.classList.toggle('active',x===button));$('stage-title').textContent=stage;render(true);});$('stages').append(button);
}
function update(s){
  state=s;
  if(clearEpoch!==null&&clearEpoch!==s.clear_epoch){events=[];selected=null;$('inspection').hidden=true;$('inspection-empty').hidden=false;$('copy').disabled=true;}
  clearEpoch=s.clear_epoch;
  $('port-state').textContent=s.connected?'COM6 listening':s.reader_mode==='released'?'COM6 released':'Reader waiting';
  $('port-dot').style.background=s.connected?'var(--green)':s.reader_mode==='released'?'#ecc071':'var(--red)';
  $('connection').textContent=s.connected?'UART LIVE':s.source_kind==='archive'?'REPLAY':'CAPTURE';
  $('source-state').textContent=s.source||'Waiting for input';
  $('subtitle').textContent=s.status||'Logging service connected';
  $('release').disabled=s.reader_mode==='released'&&!s.connected;
  $('resume').disabled=!!s.connected;
  $('buffer-count').textContent=fmt(s.retained)+' / '+fmt(s.history_limit)+' retained';
  const p=s.latest||{},legacy=p.LIVE1,c=p.codec_configuration||p.sbc_pcm||legacy||{},audio=p.sbc_pcm||{},i=p.native_i2s||{},q=p.decoder_input_queue||{},drops=p.media_drop_reasons||{};
  const channels=c.channels||audio.channels;
  $('codec').textContent=codecNames[c.codec||audio.codec||0]||'Unknown codec';
  $('format').textContent=(c.rate?fmt(c.rate/1000)+' kHz':'Rate unavailable')+(channels?' · '+(channels===2?'Stereo':channels+' channel'):'')+((c.pcm_container_bytes||audio.sample_bytes)?' · PCM container '+8*(c.pcm_container_bytes||audio.sample_bytes)+' bit':'');
  $('i2s-state').textContent=i.failed?'Fault recorded':i.running?'Running':Object.keys(i).length?'Stopped':'Waiting';
  $('i2s-state').classList.toggle('error-text',!!i.failed);
  $('i2s-detail').textContent=Object.keys(i).length?fmt(i.underrun_frames)+' underrun frames · '+fmt(i.descriptor_errors)+' descriptor errors':'No DMA snapshot received';
  $('queue').textContent=q.packet_slots?`${q.packet_slots} × ${fmt(q.packet_capacity_bytes)} B`:'—';
  $('queue-detail').textContent=q.packet_slots?fmt(drops.queue_full)+' full drops · '+fmt(q.oversize_rejects)+' size rejects':'Packet capacity / saturation';
  $('heap').textContent=audio.internal_heap_free!==undefined?(audio.internal_heap_free/1024).toFixed(1)+' KiB':'—';
  $('heap-detail').textContent=audio.internal_heap_free!==undefined?'Minimum '+(audio.internal_heap_min_free/1024).toFixed(1)+' KiB · largest '+(audio.internal_heap_largest/1024).toFixed(1)+' KiB':'Awaiting a memory snapshot';
  if(legacy&&!p.codec_configuration&&!p.sbc_pcm){
    $('codec').textContent=legacy.codec?codecNames[legacy.codec]||'Bluetooth audio':'Bluetooth audio';
    $('format').textContent=(legacy.rate?fmt(legacy.rate/1000)+' kHz':'Rate unavailable')+' · '+(legacy.ch||'?')+' channels · '+(legacy.bits||'?')+' bit reported';
    $('i2s-state').textContent=legacy.active&&legacy.started?'Playing':'Idle';$('i2s-detail').textContent=fmt(legacy.under)+' underrun counter · '+fmt(legacy.i2s_err)+' I2S errors';
    $('queue').textContent=fmt(legacy.drops)+' drops';$('queue-detail').textContent=fmt(legacy.pcm_cb)+' PCM callbacks';
    $('heap').textContent=legacy.heap!==undefined?(legacy.heap/1024).toFixed(1)+' KiB':'—';$('heap-detail').textContent=legacy.largest!==undefined?'Largest '+(legacy.largest/1024).toFixed(1)+' KiB':'Largest block unavailable';
  }
  freshness();
}
function freshness(){if(!state)return;const stamp=state.source_kind==='uart'?state.last_line_unix:state.source_updated_unix;
  if(!stamp){$('freshness').textContent='Waiting for data';return;}
  const age=Math.max(0,Date.now()/1000-stamp);$('freshness').textContent=(state.source_kind==='uart'?'UART update ':state.source_kind==='archive'?'Recorded ':'Capture written ')+new Date(stamp*1000).toLocaleTimeString()+(age>4?' · '+Math.floor(age)+'s ago':' · updating');
}
function filtered(){const query=$('search').value.toLowerCase().trim(),level=$('severity').value,current=$('source-filter').value==='current',verified=$('verified').checked;
  return events.filter(e=>(stage==='All stages'||e.stage===stage)&&(level==='all'||e.level===level)&&(!current||e.source_id===state?.source_id)&&(!verified||e.integrity==='Verified')&&(!query||e.text.toLowerCase().includes(query)||e.probe.toLowerCase().includes(query)));}
function inspect(e){selected=e;$('inspection-empty').hidden=true;$('inspection').hidden=false;$('copy').disabled=false;
  $('record-title').textContent=e.probe;$('record-badges').replaceChildren(element('span',e.level,'level '+e.level),element('span',e.integrity,'integrity '+e.integrity.toLowerCase()));
  const f=e.fields||{};const entries=[['Stage',e.stage],['Source',e.source],['Viewer time',new Date(e.time*1000).toLocaleString()],['Record ID',e.id],['ESP timer',f.at_us!==undefined?fmt(f.at_us)+' µs':'Not provided'],['Integrity',e.integrity]];
  $('record-meta').replaceChildren(...entries.flatMap(([k,v])=>[element('dt',k),element('dd',String(v))]));
  $('record-json').textContent=e.fields?JSON.stringify(e.fields,null,2):'Plain UART output — no structured fields.';$('record-raw').textContent=e.text;
  document.querySelectorAll('tr.event').forEach(row=>row.classList.toggle('selected',Number(row.dataset.id)===e.id));
}
function render(force=false){if(paused&&!force)return;if(!state)return;
  renderDirty=false;
  const matches=filtered(),shown=matches.slice(-400),fragment=document.createDocumentFragment();
  for(const e of shown){const row=element('tr',undefined,'event'+(selected?.id===e.id?' selected':''));row.dataset.id=e.id;row.tabIndex=0;
    const time=element('td',new Date(e.time*1000).toLocaleTimeString());const level=element('td');level.append(element('span',e.level,'level '+e.level));
    const probe=element('td');probe.append(element('span',e.stage,'stage'),element('span',e.probe,'probe'));const message=element('td',e.text);message.title=e.text;
    const integrity=element('td');integrity.append(element('span',e.integrity,'integrity '+e.integrity.toLowerCase()));row.append(time,level,probe,message,integrity);
    row.addEventListener('click',()=>inspect(e));row.addEventListener('keydown',event=>{if(event.key==='Enter')inspect(e);});fragment.append(row);
  }
  $('rows').replaceChildren(fragment);$('empty').hidden=shown.length>0;$('row-count').textContent=fmt(matches.length)+' matching events';
  $('stream-status').textContent=(paused?'View paused · collection continues':'Collecting in background')+' · '+fmt(state.received_lines)+' total lines · '+fmt(state.received_bytes)+' bytes'+(state.history_dropped?' · '+fmt(state.history_dropped)+' older events evicted':'');
  for(const button of document.querySelectorAll('#stages button')){const name=button.dataset.stage;const n=events.filter(e=>($('source-filter').value==='all'||e.source_id===state.source_id)&&(name==='All stages'||e.stage===name)).length;button.querySelector('.count').textContent=fmt(n);}
  if($('autoscroll').checked)$('table-wrap').scrollTop=$('table-wrap').scrollHeight;
}
async function poll(){let delay=400;try{const r=await fetch('/events?after='+cursor,{cache:'no-store'});if(!r.ok)throw Error('Service returned '+r.status);const packet=await r.json();const changed=packet.events.length>0||state?.revision!==packet.state.revision||state?.source_id!==packet.state.source_id;
    if(state&&state.pid!==packet.state.pid){events=[];cursor=0;clearEpoch=null;selected=null;update(packet.state);renderDirty=true;setTimeout(poll,0);return;}
    update(packet.state);if(packet.events.length){events.push(...packet.events);events=events.filter(e=>e.id>=packet.state.oldest_id).slice(-10000);}cursor=packet.cursor;
    if(changed)renderDirty=true;
    if(renderDirty&&Date.now()-lastRender>180){render();lastRender=Date.now();}
    if(cursor<packet.state.revision)delay=0;
  }catch(e){$('subtitle').textContent='Logging service unavailable · retrying';$('connection').textContent='OFFLINE';delay=1500;}setTimeout(poll,delay);}
$('release').addEventListener('click',()=>command('release'));
$('follow-capture').addEventListener('click',()=>command('release'));
$('resume').addEventListener('click',()=>command('resume'));
$('clear').addEventListener('click',()=>command('clear'));
$('pause').addEventListener('click',()=>{paused=!paused;$('pause').textContent=paused?'▶ Resume view':'Ⅱ Pause view';$('stream-status').textContent=paused?'View paused · capture continues in background':'Collecting in background';if(!paused)render(true);});
for(const id of ['search','severity','source-filter','verified'])$(id).addEventListener('input',()=>render(true));
$('copy').addEventListener('click',async()=>{if(!selected)return;try{await navigator.clipboard.writeText(JSON.stringify(selected.fields||selected,null,2));toast('Record copied');}catch(e){toast('Clipboard unavailable; select text in the inspector');}});
$('export').addEventListener('click',()=>{const blob=new Blob([filtered().map(e=>JSON.stringify(e)).join('\n')],{type:'application/x-ndjson'});const url=URL.createObjectURL(blob),a=element('a');a.href=url;a.download='bdk-filtered-events.ndjson';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('archive-toggle').addEventListener('click',async()=>{const panel=$('archive-panel');panel.hidden=!panel.hidden;if(panel.hidden)return;try{const captures=await(await fetch('/captures')).json();$('archives').replaceChildren(...captures.map(c=>{const o=element('option',c.label);o.value=c.path;return o;}));}catch(e){toast('Could not load capture list');}});
$('load-archive').addEventListener('click',()=>{if($('archives').value)command('replay',{path:$('archives').value});});
document.addEventListener('keydown',e=>{if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='k'){e.preventDefault();$('search').focus();}});
setInterval(freshness,1000);poll();

// Default to the complete ordered UART stream. Structured diagnostics remain
// available as a secondary view; they never decide which UART lines survive.
document.body.classList.add('raw-mode');
document.querySelector('h1').firstChild.textContent='UART console ';
const rawPanel=element('section',undefined,'raw-panel');
const rawTools=element('div',undefined,'raw-tools');
const rawSearch=element('input');rawSearch.type='search';rawSearch.placeholder='Search the UART stream…';rawSearch.setAttribute('aria-label','Search raw UART');
const rawScope=element('select');rawScope.setAttribute('aria-label','UART source');
for(const [value,text] of [['current','Current input'],['all','All retained inputs']]){const option=element('option',text);option.value=value;rawScope.append(option);}
const rawPause=element('button','Ⅱ Pause view','button subtle');
const rawDownload=element('button','↓ Download UART .txt','button subtle');
const rawFollowLabel=element('label',' Follow latest','check');
const rawFollow=element('input');rawFollow.type='checkbox';rawFollow.checked=true;rawFollowLabel.prepend(rawFollow);
rawTools.append(rawSearch,rawScope,rawPause,rawDownload,rawFollowLabel);
const rawConsole=element('div',undefined,'raw-console');rawConsole.tabIndex=0;rawConsole.setAttribute('aria-label','Continuous raw UART output');
const rawStatus=element('p','Waiting for UART lines…','raw-status');
rawPanel.append(rawTools,rawConsole,rawStatus);document.querySelector('main').insertBefore(rawPanel,document.querySelector('.metric-grid'));
const rawToggle=element('button','Structured details','button subtle');document.querySelector('.header-actions').prepend(rawToggle);
let rawAfter=0,rawSource=null,rawQuery='';
function rawUartEvents(){const query=rawSearch.value.toLowerCase();return events.filter(e=>(rawScope.value==='all'||e.source_id===state?.source_id)&&!/^MONITOR:|^Serial reader connected$/.test(e.text)&&(!query||e.text.toLowerCase().includes(query)));}
function renderRawUART(force=false){
  if(!state||paused&&!force)return;
  const query=rawSearch.value.toLowerCase();
  const sourceKey=state.pid+'|'+(rawScope.value==='all'?'all':state.source_id);
  if(force||rawSource!==sourceKey||rawQuery!==query){rawConsole.replaceChildren();rawAfter=0;rawSource=sourceKey;rawQuery=query;}
  while(rawConsole.firstChild&&Number(rawConsole.firstChild.dataset.id)<state.oldest_id)rawConsole.firstChild.remove();
  const lines=rawUartEvents(),fragment=document.createDocumentFragment();
  for(const record of lines){if(record.id<=rawAfter)continue;const line=element('span',record.text||' ','raw-line');line.dataset.id=record.id;fragment.append(line);rawAfter=record.id;}
  rawConsole.append(fragment);
  rawStatus.textContent=(paused?'View paused · collection continues':'Collecting in background')+' · '+fmt(lines.length)+' retained matching UART lines · '+(rawScope.value==='all'?'All retained inputs':state.source_kind==='uart'?'Direct COM6 UART':state.source_kind==='archive'?'Recorded capture':'Test-tool capture')+' · up to '+fmt(state.history_limit)+' records retained';
  if(rawFollow.checked)rawConsole.scrollTop=rawConsole.scrollHeight;
  renderDirty=false;
}
const structuredRender=render;
render=function(force=false){if(document.body.classList.contains('raw-mode'))renderRawUART(force);else structuredRender(force);};
rawSearch.addEventListener('input',()=>renderRawUART(true));
rawScope.addEventListener('change',()=>renderRawUART(true));
rawPause.addEventListener('click',()=>{paused=!paused;rawPause.textContent=paused?'▶ Resume view':'Ⅱ Pause view';$('pause').textContent=rawPause.textContent;if(!paused)renderRawUART(true);else rawStatus.textContent='View paused · UART collection continues in the background';});
rawFollow.addEventListener('change',()=>{if(rawFollow.checked)rawConsole.scrollTop=rawConsole.scrollHeight;});
rawDownload.addEventListener('click',()=>{const blob=new Blob([rawUartEvents().map(e=>e.text).join('\n')+'\n'],{type:'text/plain;charset=utf-8'});const url=URL.createObjectURL(blob),link=element('a');link.href=url;link.download='COM6-UART-'+new Date().toISOString().replace(/[:.]/g,'-')+'.txt';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
rawToggle.addEventListener('click',()=>{const raw=document.body.classList.toggle('raw-mode');rawToggle.textContent=raw?'Structured details':'Raw UART';render(true);});
