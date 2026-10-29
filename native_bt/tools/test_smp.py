"""Offline Core v5.4 crypto vectors; cryptography supplies independent AES/CMAC."""
import ctypes as c
import os
from pathlib import Path
import shutil
import subprocess
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.cmac import CMAC
from cryptography.hazmat.primitives.asymmetric import ec

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '_build' / 'smp'
OUT.mkdir(parents=True, exist_ok=True)
cc = os.environ.get('CC') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else shutil.which('cc'))
if not cc: raise RuntimeError('A C11 compiler is required')
binary = os.environ.get('DBT_SMP_TEST_BINARY', 'smp_test.dll')
if binary not in ('smp_test.dll','smp_worker_core.dll','smp_nvs_core.dll'):
    raise ValueError('SMP test binary must use one of the fixed workspace paths')
libpath = OUT / (binary if os.name=='nt' else binary.replace('.dll','.so'))
exports = ['c1', 's1', 'ah_resolve', 'f4', 'f5', 'f6', 'g2', 'sc_derive', 'sc_confirm', 'open', 'close', 'set_oob', 'set_passkey', 'set_max_key_size', 'set_bonding', 'start', 'receive', 'tx', 'tx_done', 'job', 'complete', 'take_stk', 'encryption', 'poll', 'dist_tx_legacy', 'dist_admitted_legacy', 'dist_final_ack_legacy', 'dist_receive_legacy', 'bond_job_legacy', 'bond_complete_legacy', 'bond_cancel_legacy']
bond_exports = ['init','close','begin','job','complete','cancel','take']
guard_exports = ['init','allowed','failure','success']
dist_exports = ['open','close','tx','tx_token','tx_admitted','final_pending','final_ack','hci_buffers_completed','receive','encryption_lost','take']
bearer_exports = ['open','close','receive','tx','tx_done','pump','dist_tx','dist_token','dist_admitted','dist_ack']
sc_exports = ['open','close','receive','poll','tx','tx_done','job','complete','take_ltk','encryption','bond_job','bond_complete','bond_cancel','set_distribution','dist_tx','dist_tx_admitted','dist_final_ack','dist_receive','numeric_value','numeric_confirm','set_passkey','set_oob','check_bond_policy']
reconnect_exports=['open','open_role','open_candidate','open_candidate_role','close','job','complete','cancel','ids','take','encryption']
sc_bearer_exports=['receive','tx','dist_token','tx_admitted','dist_admitted','dist_ack','pump','close']
feature_exports=['open','start','receive','poll','tx','tx_done','take','close','open_sc','resume_legacy','set_guard','report_result']
platform_flags=(['-Wl,/export:dbt_smp_'+s for s in exports]+['-Wl,/export:dbt_smp_bond_'+s for s in bond_exports]+['-Wl,/export:dbt_smp_guard_'+s for s in guard_exports]+['-Wl,/export:dbt_smp_dist_'+s for s in dist_exports]+['-Wl,/export:dbt_smp_bearer_'+s for s in bearer_exports]+['-Wl,/export:dbt_smp_sc_'+s for s in sc_exports]+['-Wl,/export:dbt_smp_reconnect_'+s for s in reconnect_exports]+['-Wl,/export:dbt_smp_sc_bearer_'+s for s in sc_bearer_exports]+['-Wl,/export:dbt_smp_features_'+s for s in feature_exports]) if os.name=='nt' else ['-fPIC']
san_flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('DBT_SMP_SANITIZE')=='1' and os.name!='nt' else []
build_env=os.environ.copy()
if san_flags: build_env.pop('LD_PRELOAD',None)
subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', *san_flags, '-I', str(ROOT/'include'), str(ROOT/'src/dbt_smp.c'), str(ROOT/'src/dbt_smp_crypto.c'), str(ROOT/'src/dbt_smp_bond.c'), str(ROOT/'src/dbt_smp_guard.c'), str(ROOT/'src/dbt_smp_dist.c'), str(ROOT/'src/dbt_smp_bearer.c'), str(ROOT/'src/dbt_smp_sc.c'), str(ROOT/'src/dbt_smp_features.c'), *platform_flags, '-o', str(libpath)], check=True,env=build_env)
lib = c.CDLL(str(libpath))
U8 = c.c_uint8
P = c.POINTER(U8)
def arr(b): return (U8 * len(b)).from_buffer_copy(b)
def hx(s): return bytes.fromhex(s)
def le(s): return arr(hx(s)[::-1])
AES = c.CFUNCTYPE(c.c_bool, c.c_void_p, P, P, P)
MAC = c.CFUNCTYPE(c.c_bool, c.c_void_p, P, P, c.c_uint, P)
@AES
def aes(_, k, b, out):
    e = Cipher(algorithms.AES(c.string_at(k,16)), modes.ECB()).encryptor()
    c.memmove(out, e.update(c.string_at(b,16))+e.finalize(), 16)
    return True
@MAC
def mac(_, k, m, n, out):
    v = CMAC(algorithms.AES(c.string_at(k,16))); v.update(c.string_at(m,n))
    c.memmove(out, v.finalize(), 16); return True
@MAC
def failmac(_, k, m, n, out):
    c.memset(out, 0x99, 16); return False
for name in ['dbt_smp_c1','dbt_smp_s1','dbt_smp_ah_resolve','dbt_smp_f4','dbt_smp_f5','dbt_smp_f6','dbt_smp_g2']:
    getattr(lib,name).restype = c.c_bool
lib.dbt_smp_c1.argtypes=[AES,c.c_void_p,P,P,P,P,U8,P,U8,P,P]
lib.dbt_smp_s1.argtypes=[AES,c.c_void_p,P,P,P,P]
lib.dbt_smp_ah_resolve.argtypes=[AES,c.c_void_p,P,P,c.POINTER(c.c_bool)]
lib.dbt_smp_f4.argtypes=[MAC,c.c_void_p,P,P,P,U8,P]
lib.dbt_smp_f5.argtypes=[MAC,c.c_void_p,P,P,P,P,P,P,P]
lib.dbt_smp_f6.argtypes=[MAC,c.c_void_p,P,P,P,P,P,P,P,P]
lib.dbt_smp_g2.argtypes=[MAC,c.c_void_p,P,P,P,P,c.POINTER(c.c_uint32)]
zero=arr(bytes(16)); out=arr(bytes(16))
match=c.c_bool(False)
assert lib.dbt_smp_ah_resolve(aes,None,le('ec0234a357c8ad05341010a60a397d9b'),le('7081940dfbaa'),c.byref(match)) and match.value
bad=le('7081940dfbab'); assert lib.dbt_smp_ah_resolve(aes,None,le('ec0234a357c8ad05341010a60a397d9b'),bad,c.byref(match)) and not match.value
assert not lib.dbt_smp_ah_resolve(aes,None,zero,arr(bytes(6)),c.byref(match))
assert not lib.dbt_smp_ah_resolve(aes,None,zero,arr(bytes([0,0,0,0,0,0x40])),c.byref(match))
assert not lib.dbt_smp_ah_resolve(aes,None,zero,arr(bytes([0,0,0,255,255,0x7f])),c.byref(match))
assert lib.dbt_smp_c1(aes,None,zero,le('5783d52156ad6f0e6388274ec6702ee0'),le('07071000000101'),le('05000800000302'),1,le('a1a2a3a4a5a6'),0,le('b1b2b3b4b5b6'),out)
assert bytes(out)[::-1].hex()=='1e1e3fef878988ead2a74dc5bef13b86'
assert lib.dbt_smp_s1(aes,None,zero,le('000f0e0d0c0b0a091122334455667788'),le('010203040506070899aabbccddeeff00'),out)
assert bytes(out)[::-1].hex()=='9a1fe1f0e8b0f49b5b4216ae796da062'
u=arr(hx('20b003d2f297be2c5e2c83a7e9f9a5b9eff49111acf4fddbcc0301480e359de6'))
v=arr(hx('55188b3d32f6bb9a900afcfbeed4e72a59cb9ac2f19d7cfb6b4fdd49f47fc5fd'))
n1=arr(hx('d5cb8454d177733effffb2ec712baeab')); n2=arr(hx('a6e8e7cc25a75f6e216583f7ff3dc4cf'))
a1=arr(hx('0056123737bfce')); a2=arr(hx('00a713702dcfc1'))
w=arr(hx('ec0234a357c8ad05341010a60a397d9b99796b13b4f866f1868d34f373bfa698'))
assert lib.dbt_smp_f4(mac,None,u,v,n1,0,out) and bytes(out).hex()=='f2c916f107a9bd1cf1eda1bea974872d'
mk=arr(bytes(16)); ltk=arr(bytes(16))
assert lib.dbt_smp_f5(mac,None,w,n1,n2,a1,a2,mk,ltk)
assert bytes(mk).hex()=='2965f176a1084a02fd3f6a20ce636e20'
assert bytes(ltk).hex()=='6986791169d7cd23980522b594750a38'
assert lib.dbt_smp_f6(mac,None,mk,n1,n2,arr(hx('12a3343bb453bb5408da42d20c2d0fc8')),arr(hx('010102')),a1,a2,out)
assert bytes(out).hex()=='e3c473989cd0e8c5d26c0b09da958f61'
val=c.c_uint32(); assert lib.dbt_smp_g2(mac,None,u,v,n1,n2,c.byref(val))
assert val.value == 0x2f9ed5ba
assert not lib.dbt_smp_f5(failmac,None,w,n1,n2,a1,a2,mk,ltk) and bytes(mk)==bytes(16) and bytes(ltk)==bytes(16)
print('Bluetooth Core v5.4 c1, s1, f4, f5, f6, g2 known-answer tests passed')

ECDH=c.CFUNCTYPE(c.c_bool,c.c_void_p,P,P,P)
private_keys={}
def public_bytes(private):
    p=private.public_key().public_numbers()
    return p.x.to_bytes(32,'big')+p.y.to_bytes(32,'big')
@ECDH
def ecdh(_,local,peer,out):
    local_bytes=c.string_at(local,64); peer_bytes=c.string_at(peer,64)
    try:
        private=private_keys[local_bytes]
        numbers=ec.EllipticCurvePublicNumbers(int.from_bytes(peer_bytes[:32],'big'),int.from_bytes(peer_bytes[32:],'big'),ec.SECP256R1())
        shared=private.exchange(ec.ECDH(),numbers.public_key())
        c.memmove(out,shared,32); return True
    except (ValueError,KeyError):
        return False
lib.dbt_smp_sc_derive.argtypes=[ECDH,MAC,c.c_void_p,P,P,P,P,P,P,P,P,P,P,U8,P,P,P]
lib.dbt_smp_sc_derive.restype=c.c_bool
lib.dbt_smp_sc_confirm.argtypes=[MAC,c.c_void_p,P,P,P,U8,P]
lib.dbt_smp_sc_confirm.restype=c.c_bool
priv_a=ec.derive_private_key(1,ec.SECP256R1()); priv_b=ec.derive_private_key(2,ec.SECP256R1())
pub_a=public_bytes(priv_a); pub_b=public_bytes(priv_b)
private_keys[pub_a]=priv_a; private_keys[pub_b]=priv_b
random_zero=arr(bytes(16)); iocap=arr(bytes(3))
ltk_a,ea_a,eb_a=arr(bytes(16)),arr(bytes(16)),arr(bytes(16))
ltk_b,ea_b,eb_b=arr(bytes(16)),arr(bytes(16)),arr(bytes(16))
def derive(local,peer,ltk,ea,eb):
    return lib.dbt_smp_sc_derive(ecdh,mac,None,arr(local),arr(peer),n1,n2,a1,a2,iocap,iocap,random_zero,random_zero,7,ltk,ea,eb)
assert derive(pub_a,pub_b,ltk_a,ea_a,eb_a) and derive(pub_b,pub_a,ltk_b,ea_b,eb_b)
assert bytes(ltk_a)==bytes(ltk_b) and bytes(ltk_a)[:9]==bytes(9)
assert bytes(ea_a)==bytes(ea_b) and bytes(eb_a)==bytes(eb_b) and bytes(ea_a)!=bytes(eb_a)
assert not derive(pub_a,pub_a,ltk_a,ea_a,eb_a) and bytes(ltk_a)==bytes(16)
bad_pub=bytearray(pub_b); bad_pub[-1]^=1
assert not derive(pub_a,bytes(bad_pub),ltk_a,ea_a,eb_a)
assert bytes(ltk_a)==bytes(16) and bytes(ea_a)==bytes(16) and bytes(eb_a)==bytes(16)
assert lib.dbt_smp_f4(mac,None,u,v,n1,0,out)
assert lib.dbt_smp_sc_confirm(mac,None,u,v,n1,0,out)
out[0]^=1; assert not lib.dbt_smp_sc_confirm(mac,None,u,v,n1,0,out)
print('Secure Connections P-256 worker derivation, role checks and point rejection passed')

class Job(c.Structure):
    _fields_=[('kind',c.c_int),('generation',c.c_uint32),('serial',c.c_uint32),
              ('preq',U8*7),('pres',U8*7),('ia',U8*6),('ra',U8*6),('iat',U8),('rat',U8),
              ('local_random',U8*16),('peer_random',U8*16),('tk',U8*16),('initiator',c.c_bool)]
class Result(c.Structure):
    _fields_=[('random',U8*16),('confirm',U8*16),('stk',U8*16)]
class BondRecord(c.Structure):
    _fields_=[('identity_type',U8),('identity',U8*6),('ltk',U8*16),('rand',U8*8),('irk',U8*16),
              ('ediv',c.c_uint16),('key_size',U8),('authenticated',c.c_bool),('sc',c.c_bool),('have_irk',c.c_bool),
              ('local_csrk',U8*16),('peer_csrk',U8*16),('have_local_csrk',c.c_bool),('have_peer_csrk',c.c_bool),
              ('local_irk',U8*16),('candidate_irk',U8*16),('candidate_identity',U8*6),('candidate_type',U8),
              ('have_local_irk',c.c_bool),('candidate_present',c.c_bool),('candidate_verified',c.c_bool),
              ('local_ltk',U8*16),('local_rand',U8*8),('local_ediv',c.c_uint16),('have_local_ltk',c.c_bool),('have_peer_ltk',c.c_bool)]
class BondJob(c.Structure):
    _fields_=[('generation',c.c_uint32),('serial',c.c_uint32),('op',c.c_int),('identity_type',U8),
              ('identity',U8*6),('record',BondRecord)]
class Bond(c.Structure):
    _fields_=[('generation',c.c_uint32),('serial',c.c_uint32),('job',BondJob),('loaded',BondRecord),
              ('pending',c.c_bool),('loaded_valid',c.c_bool)]
class Keys(c.Structure):
    _fields_=[('ltk',U8*16),('rand',U8*8),('irk',U8*16),('csrk',U8*16),('identity',U8*6),
              ('ediv',c.c_uint16),('identity_type',U8)]
class Dist(c.Structure):
    _fields_=[('generation',c.c_uint32),('local',Keys),('peer',Keys),('local_mask',U8),('peer_mask',U8),
              ('key_size',U8),('step',U8),('phase',U8),('tx_size',U8),('tx',U8*17),
              ('initiator',c.c_bool),('sc',c.c_bool),('failed',c.c_bool),('taken',c.c_bool),
              ('tx_serial',c.c_uint32),('final_serial',c.c_uint32),('hci_buffers_completed',c.c_uint32),('local_pdus_admitted',c.c_uint32),
              ('final_wait_ack',c.c_bool),('final_ll_acked',c.c_bool),('peer_final_received',c.c_bool),('early_final_acked',c.c_bool)]
class State(c.Structure):
    _fields_=[('handle',c.c_uint16),('generation',c.c_uint32),('serial',c.c_uint32),('deadline',c.c_uint32),
              ('state',c.c_int),('job_data',Job),('preq',U8*7),('pres',U8*7),('ia',U8*6),('ra',U8*6),
              ('iat',U8),('rat',U8),('random',U8*16),('peer_confirm',U8*16),('peer_random',U8*16),
              ('stk',U8*16),('tx_data',U8*17),('tk',U8*16),('tx_size',U8),('key_size',U8),('min_key_size',U8),('max_key_size',U8),('reason',U8),
              ('initiator',c.c_bool),('allow_just_works',c.c_bool),('peer_confirmed',c.c_bool),
              ('job_pending',c.c_bool),('key_taken',c.c_bool),('used',c.c_bool),
              ('have_oob',c.c_bool),('oob_authenticated',c.c_bool),('authenticated',c.c_bool),
              ('have_passkey',c.c_bool),('io_capability',U8),('bonding',c.c_bool),('bond_committed',c.c_bool),
              ('local_mask',U8),('peer_mask',U8),('bond',Bond),('dist',Dist),('local_distribution',Keys),('peer_distribution',Keys),
              ('peer_final_received',c.c_bool),('final_ll_acked',c.c_bool),('encryption_verified',c.c_bool),
              ('hci_buffers_completed',c.c_uint32),('local_pdus_admitted',c.c_uint32)]
S=c.POINTER(State)
lib.dbt_smp_open.argtypes=[S,c.c_uint16,c.c_uint32,U8,U8,P,U8,P,U8,c.c_bool]
lib.dbt_smp_set_oob.argtypes=[S,P,c.c_bool]
lib.dbt_smp_set_passkey.argtypes=[S,c.c_uint32,U8]
lib.dbt_smp_set_max_key_size.argtypes=[S,U8]
lib.dbt_smp_set_bonding.argtypes=[S,U8,U8,c.POINTER(Keys),c.c_bool,c.c_bool,c.c_bool,U8]
lib.dbt_smp_dist_tx_legacy.argtypes=[S,c.POINTER(P),c.POINTER(U8),c.c_uint32]
lib.dbt_smp_dist_admitted_legacy.argtypes=[S,c.c_uint32,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_dist_final_ack_legacy.argtypes=[S,c.c_uint32,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_dist_receive_legacy.argtypes=[S,P,U8,c.c_uint32]
lib.dbt_smp_bond_job_legacy.argtypes=[S,c.POINTER(BondJob)]
lib.dbt_smp_bond_complete_legacy.argtypes=[S,c.c_uint32,c.c_uint32,c.c_bool]
lib.dbt_smp_bond_cancel_legacy.argtypes=[S,c.c_uint32,c.c_uint32]
lib.dbt_smp_start.argtypes=[S,c.c_uint32]
lib.dbt_smp_receive.argtypes=[S,P,c.c_uint16,c.c_uint32]
lib.dbt_smp_tx.argtypes=[S,c.POINTER(P),c.POINTER(U8)]
lib.dbt_smp_tx_done.argtypes=[S,c.c_bool]
lib.dbt_smp_job.argtypes=[S,c.POINTER(Job)]
lib.dbt_smp_complete.argtypes=[S,c.c_uint32,c.c_uint32,c.c_bool,c.POINTER(Result),c.c_uint32]
lib.dbt_smp_take_stk.argtypes=[S,P,c.POINTER(U8)]
lib.dbt_smp_encryption.argtypes=[S,c.c_uint32,c.c_bool]
lib.dbt_smp_poll.argtypes=[S,c.c_uint32]

def transfer(src,dst,now):
    p=P(); n=U8()
    assert lib.dbt_smp_tx(c.byref(src),c.byref(p),c.byref(n))==0
    data=arr(c.string_at(p,n.value)); lib.dbt_smp_tx_done(c.byref(src),True)
    assert lib.dbt_smp_receive(c.byref(dst),data,len(data),now)==0

def work(s,rand,now):
    job=Job(); result=Result()
    assert lib.dbt_smp_job(c.byref(s),c.byref(job))==0
    value=arr(rand) if job.kind==1 else job.peer_random
    assert lib.dbt_smp_c1(aes,None,job.tk,value,job.preq,job.pres,job.iat,job.ia,job.rat,job.ra,result.confirm)
    if job.kind==1: c.memmove(result.random,value,16)
    else:
        rr=job.peer_random if job.initiator else job.local_random
        ir=job.local_random if job.initiator else job.peer_random
        assert lib.dbt_smp_s1(aes,None,job.tk,rr,ir,result.stk)
    assert lib.dbt_smp_complete(c.byref(s),job.generation+1,job.serial,True,c.byref(result),now)==2
    assert lib.dbt_smp_complete(c.byref(s),job.generation,job.serial,True,c.byref(result),now)==0

def peers():
    a,b=State(),State(); aa=arr(hx('010203040506')); bb=arr(hx('a1a2a3a4a5a6'))
    assert lib.dbt_smp_open(c.byref(a),1,77,0,0,aa,1,bb,7,True)==0
    assert lib.dbt_smp_open(c.byref(b),1,88,1,1,bb,0,aa,7,True)==0
    assert lib.dbt_smp_start(c.byref(a),1)==0
    transfer(a,b,2); work(b,bytes(range(16)),3) # worker completes before response admission
    transfer(b,a,4); work(a,bytes(range(16,32)),5)
    transfer(a,b,6); transfer(b,a,7); transfer(a,b,8)
    work(b,bytes(16),9); transfer(b,a,10); work(a,bytes(16),11)
    x,y=arr(bytes(16)),arr(bytes(16)); xs,ys=U8(),U8()
    assert lib.dbt_smp_take_stk(c.byref(a),x,c.byref(xs))==0
    assert lib.dbt_smp_take_stk(c.byref(b),y,c.byref(ys))==0
    assert bytes(x)==bytes(y) and xs.value==ys.value==16
    assert lib.dbt_smp_encryption(c.byref(a),77,True)==0
    assert lib.dbt_smp_encryption(c.byref(b),88,True)==0
    assert a.state==b.state==7
    assert lib.dbt_smp_encryption(c.byref(a),77,False)==0 and a.state==8
    return a,b

peers()
s=State(); aa=arr(bytes(6)); bb=arr(bytes([1]*6))
assert lib.dbt_smp_open(c.byref(s),1,1,0,0,aa,0,bb,16,True)==0
assert lib.dbt_smp_start(c.byref(s),0xfffffff0)==0
lib.dbt_smp_poll(c.byref(s),29984)
assert s.state==9 and s.tx_size==0
print('Legacy two-peer crypto handoff, stale generation, encryption gate, timer wrap passed')

def opened(role=1, minimum=7, allowed=True):
    s=State(); aa=arr(hx('010203040506')); bb=arr(hx('a1a2a3a4a5a6'))
    assert lib.dbt_smp_open(c.byref(s),1,123,role,0,aa,1,bb,minimum,allowed)==0
    return s
def rx(s,b,now=1):
    data=arr(bytes(b)); return lib.dbt_smp_receive(c.byref(s),data,len(data),now)
def packet(s):
    p=P(); n=U8(); assert lib.dbt_smp_tx(c.byref(s),c.byref(p),c.byref(n))==0
    return c.string_at(p,n.value)

# Reject policies and short keys before generating cryptographic material.
for fields,minimum,allowed,reason in [([1,3,0,4,16,0,0],7,True,3),
                                     ([1,3,0,0,7,0,0],16,True,6),
                                     ([1,3,0,0,16,0,0],7,False,3)]:
    t=opened(minimum=minimum,allowed=allowed); assert rx(t,fields)==0
    assert t.state==8 and packet(t)==bytes([5,reason]) and not t.job_pending

# A remote failure supersedes and clears pending local data.
t=opened(role=0); assert lib.dbt_smp_start(c.byref(t),1)==0
assert rx(t,[5,4])==0 and t.state==8 and t.tx_size==0

# Worker failure accepts no result, clears queued feature response and keys.
t=opened(); assert rx(t,[1,3,0,0,16,0,0])==0
j=Job(); assert lib.dbt_smp_job(c.byref(t),c.byref(j))==0
assert lib.dbt_smp_complete(c.byref(t),j.generation,j.serial,False,None,2)==0
assert t.state==8 and packet(t)==bytes([5,8]) and not t.job_pending
assert bytes(t.random)==bytes(16) and bytes(t.stk)==bytes(16)

# Duplicate confirm while a worker is pending is rejected and cleared.
t=opened(); assert rx(t,[1,3,0,0,16,0,0])==0
lib.dbt_smp_tx_done(c.byref(t),True)
assert rx(t,[3]+[0x42]*16,2)==0
assert rx(t,[3]+[0x42]*16,3)==0 and t.state==8 and packet(t)==bytes([5,10])
assert bytes(t.peer_confirm)==bytes(16)

# Different connection addresses on a local peer must fail confirm verification.
a,b=opened(role=0),opened(role=1)
assert lib.dbt_smp_start(c.byref(a),1)==0
transfer(a,b,2); work(b,bytes(range(16)),3); transfer(b,a,4)
work(a,bytes(range(16,32)),5); transfer(a,b,6); transfer(b,a,7); transfer(a,b,8)
work(b,bytes(16),9)
assert b.state==8 and packet(b)==bytes([5,4]) and bytes(b.stk)==bytes(16)
print('Policy, duplicate confirm, address binding and failure cleanup passed')

# A peer-provisioned, authenticated OOB TK drives both c1 and s1.
a,b=State(),State(); aa=arr(hx('010203040506')); bb=arr(hx('a1a2a3a4a5a6'))
assert lib.dbt_smp_open(c.byref(a),1,51,0,0,aa,1,bb,7,False)==0
assert lib.dbt_smp_open(c.byref(b),1,52,1,1,bb,0,aa,7,False)==0
tk=arr(hx('0102030405060708090a0b0c0d0e0f10'))
assert lib.dbt_smp_set_oob(c.byref(a),tk,True)==0
assert lib.dbt_smp_set_oob(c.byref(b),tk,True)==0
assert lib.dbt_smp_set_max_key_size(c.byref(b),7)==0
assert lib.dbt_smp_start(c.byref(a),1)==0
transfer(a,b,2); work(b,bytes(range(16)),3); transfer(b,a,4)
work(a,bytes(range(16,32)),5); transfer(a,b,6); transfer(b,a,7); transfer(a,b,8)
work(b,bytes(16),9); transfer(b,a,10); work(a,bytes(16),11)
x,y=arr(bytes(16)),arr(bytes(16)); xs,ys=U8(),U8()
assert lib.dbt_smp_take_stk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_take_stk(c.byref(b),y,c.byref(ys))==0
assert bytes(x)==bytes(y) and xs.value==ys.value==7 and bytes(x)[7:]==bytes(9)
assert bytes(a.tk)==bytes(16) and bytes(b.tk)==bytes(16)
assert lib.dbt_smp_encryption(c.byref(a),51,True)==0 and a.authenticated
assert lib.dbt_smp_encryption(c.byref(b),52,True)==0 and b.authenticated
assert lib.dbt_smp_encryption(c.byref(a),51,False)==0 and not a.authenticated

# Configured OOB never silently downgrades, and a nonauthenticated OOB channel
# cannot satisfy a peer's explicit MITM requirement.
t=opened(allowed=True); assert lib.dbt_smp_set_oob(c.byref(t),tk,True)==0
assert rx(t,[1,3,0,0,16,0,0])==0 and packet(t)==bytes([5,2])
t=opened(); assert lib.dbt_smp_set_oob(c.byref(t),tk,False)==0
assert rx(t,[1,3,1,4,16,0,0])==0 and packet(t)==bytes([5,3])
print('Legacy OOB pairing, authentication policy and downgrade rejection passed')

a,b=State(),State(); aa=arr(hx('010203040506')); bb=arr(hx('a1a2a3a4a5a6'))
assert lib.dbt_smp_open(c.byref(a),1,51,0,0,aa,1,bb,7,False)==0
assert lib.dbt_smp_open(c.byref(b),1,52,1,1,bb,0,aa,7,False)==0
assert lib.dbt_smp_set_passkey(c.byref(a),19655,2)==0
assert lib.dbt_smp_set_passkey(c.byref(b),19655,0)==0
assert bytes(a.tk)[:3]==hx('c74c00')
assert lib.dbt_smp_start(c.byref(a),1)==0
transfer(a,b,2); work(b,bytes(range(16)),3); transfer(b,a,4)
work(a,bytes(range(16,32)),5); transfer(a,b,6); transfer(b,a,7); transfer(a,b,8)
work(b,bytes(16),9); transfer(b,a,10); work(a,bytes(16),11)
x,y=arr(bytes(16)),arr(bytes(16)); xs,ys=U8(),U8()
assert lib.dbt_smp_take_stk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_take_stk(c.byref(b),y,c.byref(ys))==0 and bytes(x)==bytes(y)
assert lib.dbt_smp_encryption(c.byref(a),51,True)==0 and a.authenticated
assert lib.dbt_smp_encryption(c.byref(b),52,True)==0 and b.authenticated
t=opened(); assert lib.dbt_smp_set_passkey(c.byref(t),19655,0)==0
assert rx(t,[1,3,0,4,16,0,0])==0 and t.state==8 and packet(t)==bytes([5,3])
print('Legacy Passkey Entry paired exchange and incompatible I/O rejection passed')

B=c.POINTER(Bond); R=c.POINTER(BondRecord)
lib.dbt_smp_bond_init.argtypes=[B,c.c_uint32]
lib.dbt_smp_bond_begin.argtypes=[B,c.c_int,U8,P,R]
lib.dbt_smp_bond_job.argtypes=[B,c.POINTER(BondJob)]
lib.dbt_smp_bond_complete.argtypes=[B,c.c_uint32,c.c_uint32,c.c_bool,R]
lib.dbt_smp_bond_cancel.argtypes=[B,c.c_uint32,c.c_uint32]
lib.dbt_smp_bond_take.argtypes=[B,U8,c.c_bool,c.c_bool,R]
bond=Bond(); lib.dbt_smp_bond_init(c.byref(bond),13)
identity=arr(hx('010203040506')); record=BondRecord(); record.identity_type=0
c.memmove(record.identity,identity,6); record.key_size=16; record.ltk[0]=1
record.authenticated=True; record.sc=True
assert lib.dbt_smp_bond_begin(c.byref(bond),1,0,identity,c.byref(record))==0
job=BondJob(); assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
copy=BondRecord(); assert lib.dbt_smp_bond_take(c.byref(bond),7,False,False,c.byref(copy))==5
assert lib.dbt_smp_bond_complete(c.byref(bond),14,job.serial,True,None)==2
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,True,None)==0
assert lib.dbt_smp_bond_begin(c.byref(bond),2,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,True,c.byref(record))==0
assert lib.dbt_smp_bond_take(c.byref(bond),16,True,True,c.byref(copy))==0
assert bytes(copy.ltk)==bytes(record.ltk)
assert lib.dbt_smp_bond_take(c.byref(bond),7,False,False,c.byref(copy))==1
assert lib.dbt_smp_bond_begin(c.byref(bond),3,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,False,None)==2
assert lib.dbt_smp_bond_take(c.byref(bond),7,False,False,c.byref(copy))==1
assert lib.dbt_smp_bond_begin(c.byref(bond),2,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
record.authenticated=False
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,True,c.byref(record))==0
assert lib.dbt_smp_bond_take(c.byref(bond),7,True,False,c.byref(copy))==4
assert lib.dbt_smp_bond_begin(c.byref(bond),2,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
record.identity[0]^=1
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,True,c.byref(record))==2
assert lib.dbt_smp_bond_take(c.byref(bond),7,False,False,c.byref(copy))==1
record.identity[0]^=1; record.key_size=7; record.ltk[15]=1
assert lib.dbt_smp_bond_begin(c.byref(bond),1,0,identity,c.byref(record))==2
assert lib.dbt_smp_bond_begin(c.byref(bond),3,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,True,None)==0
assert lib.dbt_smp_bond_begin(c.byref(bond),2,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,False,None)==2
assert lib.dbt_smp_bond_take(c.byref(bond),7,False,False,c.byref(copy))==1
assert lib.dbt_smp_bond_begin(c.byref(bond),2,0,identity,None)==0
assert lib.dbt_smp_bond_job(c.byref(bond),c.byref(job))==0
assert lib.dbt_smp_bond_cancel(c.byref(bond),13,job.serial)==0
assert lib.dbt_smp_bond_complete(c.byref(bond),13,job.serial,True,c.byref(record))==2
print('Bond broker store/load/delete acknowledgments, identity, stale generation and policy passed')

class Guard(c.Structure):
    _fields_=[('base',c.c_uint32),('max',c.c_uint32),('decay',c.c_uint32),('deadline',c.c_uint32),
              ('last_decay',c.c_uint32),('failures',U8),('waiting',c.c_bool)]
G=c.POINTER(Guard)
lib.dbt_smp_guard_init.argtypes=[G,c.c_uint32,c.c_uint32,c.c_uint32,c.c_uint32]
lib.dbt_smp_guard_allowed.argtypes=[G,c.c_uint32]; lib.dbt_smp_guard_allowed.restype=c.c_bool
lib.dbt_smp_guard_failure.argtypes=[G,c.c_uint32]
lib.dbt_smp_guard_success.argtypes=[G,c.c_uint32]
g=Guard(); assert lib.dbt_smp_guard_init(c.byref(g),100,400,1000,0xfffffff0)==0
lib.dbt_smp_guard_failure(c.byref(g),0xfffffff0)
assert not lib.dbt_smp_guard_allowed(c.byref(g),83)
assert lib.dbt_smp_guard_allowed(c.byref(g),84)
lib.dbt_smp_guard_failure(c.byref(g),84)
assert not lib.dbt_smp_guard_allowed(c.byref(g),283)
assert lib.dbt_smp_guard_allowed(c.byref(g),284)
lib.dbt_smp_guard_failure(c.byref(g),284)
assert not lib.dbt_smp_guard_allowed(c.byref(g),683)
assert lib.dbt_smp_guard_allowed(c.byref(g),684)
assert lib.dbt_smp_guard_allowed(c.byref(g),3284) and g.failures==0
lib.dbt_smp_guard_failure(c.byref(g),3284)
lib.dbt_smp_guard_success(c.byref(g),3285)
assert lib.dbt_smp_guard_allowed(c.byref(g),3285) and g.failures==0
print('Repeated-failure exponential delay, decay and clock wrap passed')

D=c.POINTER(Dist); K=c.POINTER(Keys)
lib.dbt_smp_dist_open.argtypes=[D,c.c_uint32,c.c_bool,c.c_bool,U8,U8,U8,c.c_bool,K]
lib.dbt_smp_dist_tx.argtypes=[D,c.POINTER(P),c.POINTER(U8)]
lib.dbt_smp_dist_tx_token.argtypes=[D,c.POINTER(c.c_uint32)]
lib.dbt_smp_dist_tx_admitted.argtypes=[D,c.c_uint32,c.c_uint32,c.c_bool]
lib.dbt_smp_dist_final_pending.argtypes=[D,c.POINTER(c.c_uint32)]
lib.dbt_smp_dist_final_ack.argtypes=[D,c.c_uint32,c.c_uint32,c.c_bool]
lib.dbt_smp_dist_hci_buffers_completed.argtypes=[D,c.c_uint32,c.c_uint16]
lib.dbt_smp_dist_receive.argtypes=[D,P,U8]
lib.dbt_smp_dist_encryption_lost.argtypes=[D,c.c_uint32]
lib.dbt_smp_dist_take.argtypes=[D,K]
def dist_transfer(sender,receiver):
    p=P(); n=U8(); assert lib.dbt_smp_dist_tx(c.byref(sender),c.byref(p),c.byref(n))==0
    tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(sender),c.byref(tx_token))==0
    packet=arr(c.string_at(p,n.value)); assert lib.dbt_smp_dist_receive(c.byref(receiver),packet,n.value)==0
    assert lib.dbt_smp_dist_tx_admitted(c.byref(sender),sender.generation,tx_token.value,True)==0
    token=c.c_uint32()
    if lib.dbt_smp_dist_final_pending(c.byref(sender),c.byref(token))==0:
        assert lib.dbt_smp_dist_final_ack(c.byref(sender),sender.generation,token.value,True)==0
    return packet[0]
ka,kb=Keys(),Keys()
for i in range(7): ka.ltk[i]=i+1; kb.ltk[i]=i+11
for i in range(16): ka.irk[i]=i+1; kb.irk[i]=i+21; ka.csrk[i]=i+2; kb.csrk[i]=i+31
ka.identity_type=0; kb.identity_type=1; kb.identity[5]=0xc0
ka.ediv=0x1234; kb.ediv=0x5678
ca,cb=Dist(),Dist()
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,False,7,7,7,True,c.byref(ka))==0
assert lib.dbt_smp_dist_open(c.byref(cb),102,False,False,7,7,7,True,c.byref(kb))==0
assert [dist_transfer(cb,ca) for _ in range(5)]==[6,7,8,9,10]
assert [dist_transfer(ca,cb) for _ in range(5)]==[6,7,8,9,10]
peer_a,peer_b=Keys(),Keys()
assert lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==0
assert lib.dbt_smp_dist_take(c.byref(cb),c.byref(peer_b))==0
assert bytes(peer_a.ltk)==bytes(kb.ltk) and bytes(peer_b.ltk)==bytes(ka.ltk)
assert peer_a.ediv==kb.ediv and bytes(peer_a.irk)==bytes(kb.irk)
assert lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==5
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,True,16,1,0,True,c.byref(ka))==2
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,False,7,0,1,False,None)==2
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,False,7,0,1,True,None)==0
bad=arr(bytes([7]+[0]*10)); assert lib.dbt_smp_dist_receive(c.byref(ca),bad,len(bad))==2
assert ca.failed and lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==5
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,False,7,0,1,True,None)==0
assert lib.dbt_smp_dist_encryption_lost(c.byref(ca),102)==2
assert lib.dbt_smp_dist_encryption_lost(c.byref(ca),101)==0 and ca.failed
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,True,16,0,2,True,None)==0
bad=arr(bytes([8]+[3]*16)); assert lib.dbt_smp_dist_receive(c.byref(ca),bad,len(bad))==0
bad=arr(bytes([9,1,0,0,0,0,0,0])); assert lib.dbt_smp_dist_receive(c.byref(ca),bad,len(bad))==2
assert ca.failed and bytes(ca.peer.irk)==bytes(16)
assert lib.dbt_smp_dist_open(c.byref(cb),102,False,False,7,1,0,True,c.byref(kb))==0
p=P(); n=U8(); assert lib.dbt_smp_dist_tx(c.byref(cb),c.byref(p),c.byref(n))==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(cb),c.byref(tx_token))==0
assert lib.dbt_smp_dist_tx_admitted(c.byref(cb),102,tx_token.value,False)==0
assert cb.failed and bytes(cb.local.ltk)==bytes(16) and bytes(cb.tx)==bytes(17)
assert lib.dbt_smp_dist_open(c.byref(ca),101,True,False,7,0,1,True,None)==0
bad=arr(bytes([6]+[1]*16)); assert lib.dbt_smp_dist_receive(c.byref(ca),bad,len(bad))==2
assert ca.failed and bytes(ca.peer.ltk)==bytes(16)
# Local final key requires its own correlated ACK, even if HCI frees buffers.
assert lib.dbt_smp_dist_open(c.byref(cb),103,False,True,16,4,0,True,c.byref(kb))==0
p=P(); n=U8(); assert lib.dbt_smp_dist_tx(c.byref(cb),c.byref(p),c.byref(n))==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(cb),c.byref(tx_token))==0
assert lib.dbt_smp_dist_tx_admitted(c.byref(cb),103,tx_token.value,True)==0 and cb.final_wait_ack
assert lib.dbt_smp_dist_hci_buffers_completed(c.byref(cb),103,1)==0
assert cb.hci_buffers_completed==1 and not cb.final_ll_acked
assert lib.dbt_smp_dist_take(c.byref(cb),c.byref(peer_a))==5
token=c.c_uint32(); assert lib.dbt_smp_dist_final_pending(c.byref(cb),c.byref(token))==0
assert lib.dbt_smp_dist_final_ack(c.byref(cb),104,token.value,True)==2
assert lib.dbt_smp_dist_final_ack(c.byref(cb),103,token.value,True)==0
assert cb.final_ll_acked and not cb.peer_final_received
assert lib.dbt_smp_dist_take(c.byref(cb),c.byref(peer_a))==0
# Conversely a receiver completes on its validated peer-final key, without a
# fabricated local link-layer acknowledgment.
assert lib.dbt_smp_dist_open(c.byref(ca),105,True,True,16,0,4,True,None)==0
last=arr(bytes([10])+bytes([0x66]*16))
assert lib.dbt_smp_dist_receive(c.byref(ca),last,len(last))==0
assert ca.peer_final_received and not ca.final_ll_acked
assert lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==0
# A real final ACK may race ahead of the admission callback. It is retained but
# cannot complete the phase until matching admission; stale tokens never apply.
assert lib.dbt_smp_dist_open(c.byref(ca),106,True,True,16,4,0,True,c.byref(ka))==0
assert lib.dbt_smp_dist_tx(c.byref(ca),c.byref(p),c.byref(n))==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(ca),c.byref(tx_token))==0
assert lib.dbt_smp_dist_final_ack(c.byref(ca),107,tx_token.value,True)==2
assert lib.dbt_smp_dist_final_ack(c.byref(ca),106,tx_token.value,True)==0
assert ca.early_final_acked and not ca.final_ll_acked and lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==5
assert lib.dbt_smp_dist_final_ack(c.byref(ca),106,tx_token.value,True)==2
assert lib.dbt_smp_dist_tx_admitted(c.byref(ca),106,tx_token.value+1,True)==2
assert lib.dbt_smp_dist_tx_admitted(c.byref(ca),106,tx_token.value,True)==0
assert ca.final_ll_acked and lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==0
assert lib.dbt_smp_dist_open(c.byref(ca),107,True,True,16,4,0,True,c.byref(ka))==0
assert lib.dbt_smp_dist_tx(c.byref(ca),c.byref(p),c.byref(n))==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(ca),c.byref(tx_token))==0
assert lib.dbt_smp_dist_final_ack(c.byref(ca),107,tx_token.value,True)==0
assert lib.dbt_smp_dist_tx_admitted(c.byref(ca),107,tx_token.value,False)==0
assert ca.failed and lib.dbt_smp_dist_take(c.byref(ca),c.byref(peer_a))==5
print('Key distribution peer ordering, encrypted gate, key bounds and cleanup passed')

class Bearer(c.Structure): _fields_=[('engine',State),('pending',U8*17),('pending_size',U8)]
BP=c.POINTER(Bearer)
lib.dbt_smp_bearer_open.argtypes=[BP,c.c_uint16,c.c_uint32,U8,U8,P,U8,P,U8,c.c_bool]
lib.dbt_smp_bearer_receive.argtypes=[BP,c.c_uint16,c.c_uint32,c.c_uint16,P,c.c_uint16,c.c_uint32]
lib.dbt_smp_bearer_tx.argtypes=[BP,c.c_uint16,c.c_uint32,c.POINTER(P),c.POINTER(U8)]
lib.dbt_smp_bearer_tx_done.argtypes=[BP,c.c_uint16,c.c_uint32,c.c_bool]
bearer=Bearer(); assert lib.dbt_smp_bearer_open(c.byref(bearer),4,29,1,0,aa,1,bb,7,True)==0
req=arr(bytes([1,3,0,0,16,0,0])); p=P(); n=U8()
assert lib.dbt_smp_bearer_receive(c.byref(bearer),4,28,6,req,7,1)==2
assert lib.dbt_smp_bearer_receive(c.byref(bearer),4,29,4,req,7,1)==2
assert bearer.engine.state==0
assert lib.dbt_smp_bearer_receive(c.byref(bearer),4,29,6,req,7,1)==0
assert lib.dbt_smp_bearer_tx(c.byref(bearer),4,29,c.byref(p),c.byref(n))==0 and n.value==7
assert lib.dbt_smp_bearer_tx_done(c.byref(bearer),4,28,True)==2 and bearer.engine.tx_size==7
assert lib.dbt_smp_bearer_tx_done(c.byref(bearer),4,29,True)==0 and bearer.engine.tx_size==0
print('Owner-only LE fixed-channel adapter rejects wrong CID and stale generations')

class SCJob(c.Structure):
    _fields_=[('kind',c.c_int),('generation',c.c_uint32),('serial',c.c_uint32),('initiator',c.c_bool),
              ('key_size',U8),('preq',U8*7),('pres',U8*7),('iat',U8),('rat',U8),('ia',U8*6),('ra',U8*6),
              ('local_public',U8*64),('peer_public',U8*64),('local_nonce',U8*16),('peer_nonce',U8*16),('peer_confirm',U8*16),
              ('passkey',c.c_uint32),('round',U8),('local_oob_random',U8*16),('peer_oob_random',U8*16),('peer_oob_confirm',U8*16)]
class SCResult(c.Structure):
    _fields_=[('public_key',U8*64),('nonce',U8*16),('confirm',U8*16),('ltk',U8*16),
              ('local_check',U8*16),('peer_check',U8*16),('peer_commitment_valid',c.c_bool),('numeric_value',c.c_uint32)]
class SC(c.Structure):
    _fields_=[('handle',c.c_uint16),('generation',c.c_uint32),('serial',c.c_uint32),('deadline',c.c_uint32),
              ('state',c.c_int),('job_data',SCJob),('preq',U8*7),('pres',U8*7),('iat',U8),('rat',U8),
              ('ia',U8*6),('ra',U8*6),('key_size',U8),('local_public',U8*64),('peer_public',U8*64),
              ('local_nonce',U8*16),('peer_nonce',U8*16),('peer_confirm',U8*16),('ltk',U8*16),
              ('local_check',U8*16),('peer_check',U8*16),('tx_data',U8*65),('tx_size',U8),
              ('initiator',c.c_bool),('job_pending',c.c_bool),('key_taken',c.c_bool),('used',c.c_bool),
              ('bonding',c.c_bool),('bond_committed',c.c_bool),('bond_ltk',U8*16),('bond',Bond),
              ('local_mask',U8),('peer_mask',U8),('local_csrk',U8*16),('peer_csrk',U8*16),
              ('distribution_ready',c.c_bool),('dist',Dist),('numeric',c.c_bool),('authenticated',c.c_bool),
              ('numeric_value',c.c_uint32),('passkey',c.c_uint32),('round',U8),
              ('passkey_mode',c.c_bool),('passkey_ready',c.c_bool),('oob_mode',c.c_bool),('oob_ready',c.c_bool),
              ('oob_authenticated',c.c_bool),('local_oob_public',U8*64),('local_oob_random',U8*16),
              ('peer_oob_random',U8*16),('peer_oob_confirm',U8*16),('bond_policy_checked',c.c_bool),
              ('local_distribution',Keys),('peer_distribution',Keys),
              ('peer_final_received',c.c_bool),('final_ll_acked',c.c_bool),('encryption_verified',c.c_bool),
              ('hci_buffers_completed',c.c_uint32),('local_pdus_admitted',c.c_uint32)]
SP=c.POINTER(SC)
lib.dbt_smp_sc_open.argtypes=[SP,c.c_uint16,c.c_uint32,U8,U8,P,U8,P,P,P,U8,c.c_uint32]
lib.dbt_smp_sc_job.argtypes=[SP,c.POINTER(SCJob)]
lib.dbt_smp_sc_complete.argtypes=[SP,c.c_uint32,c.c_uint32,c.c_bool,c.POINTER(SCResult),c.c_uint32]
lib.dbt_smp_sc_tx.argtypes=[SP,c.POINTER(P),c.POINTER(U8)]
lib.dbt_smp_sc_tx_done.argtypes=[SP,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_receive.argtypes=[SP,P,c.c_uint16,c.c_uint32]
lib.dbt_smp_sc_take_ltk.argtypes=[SP,P,c.POINTER(U8)]
lib.dbt_smp_sc_encryption.argtypes=[SP,c.c_uint32,c.c_bool]
lib.dbt_smp_sc_poll.argtypes=[SP,c.c_uint32]
lib.dbt_smp_sc_bond_job.argtypes=[SP,c.POINTER(BondJob)]
lib.dbt_smp_sc_bond_complete.argtypes=[SP,c.c_uint32,c.c_uint32,c.c_bool]
lib.dbt_smp_sc_bond_cancel.argtypes=[SP,c.c_uint32,c.c_uint32]
lib.dbt_smp_sc_set_distribution.argtypes=[SP,K]
lib.dbt_smp_sc_dist_tx.argtypes=[SP,c.POINTER(P),c.POINTER(U8),c.c_uint32]
lib.dbt_smp_sc_dist_tx_admitted.argtypes=[SP,c.c_uint32,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_dist_final_ack.argtypes=[SP,c.c_uint32,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_dist_receive.argtypes=[SP,P,U8,c.c_uint32]
lib.dbt_smp_sc_numeric_value.argtypes=[SP,c.c_uint32,c.POINTER(c.c_uint32)]
lib.dbt_smp_sc_numeric_confirm.argtypes=[SP,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_set_passkey.argtypes=[SP,c.c_uint32]
lib.dbt_smp_sc_set_oob.argtypes=[SP,P,P,P,P,c.c_bool]
lib.dbt_smp_sc_check_bond_policy.argtypes=[SP,c.c_bool,c.c_bool,U8]
def sc_wire_pub(be): return be[:32][::-1]+be[32:][::-1]
def sc_work(s,now):
    j=SCJob(); r=SCResult(); assert lib.dbt_smp_sc_job(c.byref(s),c.byref(j))==0
    if j.kind==1:
        pub=pub_a if j.initiator else pub_b
        c.memmove(r.public_key,sc_wire_pub(pub),64)
        c.memmove(r.nonce,bytes(range(16,32)) if j.initiator else bytes(range(16)),16)
    elif j.kind==2:
        pkb=arr(bytes(j.local_public)[:32][::-1]); pka=arr(bytes(j.peer_public)[:32][::-1])
        nb=arr(bytes(j.local_nonce)[::-1]); cfm=arr(bytes(16))
        assert lib.dbt_smp_f4(mac,None,pkb,pka,nb,0,cfm)
        c.memmove(r.confirm,bytes(cfm)[::-1],16)
    elif j.kind==6:
        peer_x=arr(bytes(j.peer_public)[:32][::-1])
        r.peer_commitment_valid=lib.dbt_smp_sc_confirm(mac,None,peer_x,peer_x,
            arr(bytes(j.peer_oob_random)[::-1]),0,arr(bytes(j.peer_oob_confirm)[::-1]))
    elif j.kind in (4,5):
        local_x=arr(bytes(j.local_public)[:32][::-1]); peer_x=arr(bytes(j.peer_public)[:32][::-1])
        z=0x80|((j.passkey>>j.round)&1)
        if j.kind==4:
            nonce=bytes([j.round+1+(0 if j.initiator else 64)]*16)
            c.memmove(r.nonce,nonce,16); cfm=arr(bytes(16))
            assert lib.dbt_smp_f4(mac,None,local_x,peer_x,arr(nonce[::-1]),z,cfm)
            c.memmove(r.confirm,bytes(cfm)[::-1],16)
        else:
            r.peer_commitment_valid=lib.dbt_smp_sc_confirm(mac,None,peer_x,local_x,
                arr(bytes(j.peer_nonce)[::-1]),z,arr(bytes(j.peer_confirm)[::-1]))
    else:
        local=arr(sc_wire_pub(bytes(j.local_public))); peer=arr(sc_wire_pub(bytes(j.peer_public)))
        na=arr(bytes(j.local_nonce if j.initiator else j.peer_nonce)[::-1])
        nb=arr(bytes(j.peer_nonce if j.initiator else j.local_nonce)[::-1])
        a=arr(bytes([j.iat])+bytes(j.ia)[::-1]); b=arr(bytes([j.rat])+bytes(j.ra)[::-1])
        ioa=arr(bytes(j.preq[1:4])[::-1]); iob=arr(bytes(j.pres[1:4])[::-1])
        ltk,ea,eb=arr(bytes(16)),arr(bytes(16)),arr(bytes(16))
        oob_method=bool(j.preq[2] or j.pres[2])
        passkey_method=not oob_method and ((j.preq[3]|j.pres[3])&4) and not (j.preq[1] in (1,4) and j.pres[1] in (1,4))
        rvalue=arr(j.passkey.to_bytes(16,'big')) if passkey_method else random_zero
        ra=arr(bytes(j.local_oob_random if j.initiator else j.peer_oob_random)[::-1]) if oob_method else rvalue
        rb=arr(bytes(j.peer_oob_random if j.initiator else j.local_oob_random)[::-1]) if oob_method else rvalue
        ok=lib.dbt_smp_sc_derive(ecdh,mac,None,local,peer,na,nb,a,b,ioa,iob,ra,rb,j.key_size,ltk,ea,eb)
        assert ok
        c.memmove(r.ltk,bytes(ltk)[::-1],16)
        c.memmove(r.local_check,bytes(ea if j.initiator else eb)[::-1],16)
        c.memmove(r.peer_check,bytes(eb if j.initiator else ea)[::-1],16)
        if j.initiator and not passkey_method and not oob_method:
            r.peer_commitment_valid=lib.dbt_smp_sc_confirm(mac,None,arr(bytes(j.peer_public)[:32][::-1]),
                arr(bytes(j.local_public)[:32][::-1]),nb,0,arr(bytes(j.peer_confirm)[::-1]))
        if ((j.preq[3]|j.pres[3])&4) and not passkey_method and not oob_method:
            pka=arr(bytes(j.local_public if j.initiator else j.peer_public)[:32][::-1])
            pkb=arr(bytes(j.peer_public if j.initiator else j.local_public)[:32][::-1])
            numeric=c.c_uint32(); assert lib.dbt_smp_g2(mac,None,pka,pkb,na,nb,c.byref(numeric))
            r.numeric_value=numeric.value%1000000
    assert lib.dbt_smp_sc_complete(c.byref(s),j.generation+1,j.serial,True,c.byref(r),now)==2
    assert lib.dbt_smp_sc_complete(c.byref(s),j.generation,j.serial,True,c.byref(r),now)==0

def sc_transfer(src,dst,now,tamper=False):
    p=P(); n=U8(); assert lib.dbt_smp_sc_tx(c.byref(src),c.byref(p),c.byref(n))==0
    data=bytearray(c.string_at(p,n.value)); lib.dbt_smp_sc_tx_done(c.byref(src),True,now)
    if tamper: data[1]^=1
    packet=arr(data); assert lib.dbt_smp_sc_receive(c.byref(dst),packet,len(data),now)==0
    return data[0]
def sc_setup(bonding=False,signing=False,numeric=False,passkeys=None,oob=False,identity_keys=False):
    flags=(9 if bonding else 8) | (4 if numeric or passkeys or oob else 0)
    mask=(4 if signing else 0)|(2 if identity_keys else 0)
    ioa=1 if numeric else 2 if passkeys else 3
    iob=1 if numeric else 0 if passkeys else 3
    req=arr(bytes([1,ioa,int(oob),flags,16,mask,mask])); rsp=arr(bytes([2,iob,int(oob),flags,7,mask,mask]))
    a,b=SC(),SC(); ia=arr(hx('010203040506')); ib=arr(hx('a1a2a3a4a5e6'))
    assert lib.dbt_smp_sc_open(c.byref(a),1,201,0,0,ia,1,ib,req,rsp,7,1)==0
    assert lib.dbt_smp_sc_open(c.byref(b),2,202,1,1,ib,0,ia,req,rsp,7,1)==0
    if passkeys:
        assert lib.dbt_smp_sc_set_passkey(c.byref(a),passkeys[0])==0
        assert lib.dbt_smp_sc_set_passkey(c.byref(b),passkeys[1])==0
    if oob:
        ra=arr(bytes([0x71]*16)); rb=arr(bytes([0x82]*16)); ca=arr(bytes(16)); cb=arr(bytes(16))
        pka=arr(pub_a[:32]); pkb=arr(pub_b[:32])
        assert lib.dbt_smp_f4(mac,None,pka,pka,arr(bytes(ra)[::-1]),0,ca)
        assert lib.dbt_smp_f4(mac,None,pkb,pkb,arr(bytes(rb)[::-1]),0,cb)
        assert lib.dbt_smp_sc_set_oob(c.byref(a),arr(sc_wire_pub(pub_a)),ra,rb,arr(bytes(cb)[::-1]),False)==4
        assert lib.dbt_smp_sc_set_oob(c.byref(a),arr(sc_wire_pub(pub_a)),ra,rb,arr(bytes(cb)[::-1]),True)==0
        assert lib.dbt_smp_sc_set_oob(c.byref(b),arr(sc_wire_pub(pub_b)),rb,ra,arr(bytes(ca)[::-1]),True)==0
    if bonding:
        assert lib.dbt_smp_sc_check_bond_policy(c.byref(a),False,False,0)==0
        assert lib.dbt_smp_sc_check_bond_policy(c.byref(b),False,False,0)==0
    if signing or identity_keys:
        ka,kb=Keys(),Keys(); c.memmove(ka.csrk,bytes([0xA1]*16),16); c.memmove(kb.csrk,bytes([0xB2]*16),16)
        c.memmove(ka.irk,bytes([0x13]*16),16); c.memmove(kb.irk,bytes([0x24]*16),16)
        ka.identity_type=0; c.memmove(ka.identity,hx('111213141516'),6)
        kb.identity_type=1; c.memmove(kb.identity,hx('2122232425c6'),6)
        assert lib.dbt_smp_sc_set_distribution(c.byref(a),c.byref(ka))==0
        assert lib.dbt_smp_sc_set_distribution(c.byref(b),c.byref(kb))==0
    sc_work(a,2); sc_work(b,2)
    assert sc_transfer(a,b,3)==12; assert sc_transfer(b,a,4)==12
    if not passkeys and not oob: sc_work(b,5)
    return a,b
# Bonding an RPA without a negotiated peer identity is explicitly unsupported.
rpa_no_identity=SC(); req=arr(bytes([1,3,0,9,16,0,0])); rsp=arr(bytes([2,3,0,9,16,0,0]))
assert lib.dbt_smp_sc_open(c.byref(rpa_no_identity),1,90,1,0,arr(hx('010203040506')),
                            1,le('7081940dfbaa'),req,rsp,7,0)==4
a,b=sc_setup()
assert sc_transfer(b,a,6)==3; assert sc_transfer(a,b,7)==4; assert sc_transfer(b,a,8)==4
sc_work(b,9); sc_work(a,9)
assert sc_transfer(a,b,10)==13; assert sc_transfer(b,a,11)==13
x,y=arr(bytes(16)),arr(bytes(16)); xs,ys=U8(),U8()
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0
assert bytes(x)==bytes(y) and xs.value==ys.value==7 and bytes(x)[7:]==bytes(9)
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_encryption(c.byref(b),202,True)==0
assert a.state==b.state==9
a,b=sc_setup(); sc_transfer(b,a,6,tamper=True); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(a,9); assert a.state==10 and a.tx_data[0]==5 and a.tx_data[1]==4
a,b=sc_setup(); sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9); sc_transfer(a,b,10,tamper=True)
assert b.state==10 and b.tx_data[0]==5 and b.tx_data[1]==11
a,b=sc_setup(); lib.dbt_smp_sc_poll(c.byref(a),30005)
assert a.state==11 and not a.job_pending and a.tx_size==0
print('Two-peer SC Just Works exchange, check tampering, key size and timeout passed')

class Reconnect(c.Structure):
    _fields_=[('generation',c.c_uint32),('broker',Bond),('record',BondRecord),('state',c.c_int),
              ('min_key_size',U8),('require_authenticated',c.c_bool),('require_sc',c.c_bool),
              ('candidate_mode',c.c_bool),('candidate_type',U8),('candidate_identity',U8*6),('local_role_peripheral',c.c_bool),
              ('peer_key_verified',c.c_bool),('identity_promotion_committed',c.c_bool)]
RP=c.POINTER(Reconnect)
lib.dbt_smp_reconnect_open.argtypes=[RP,c.c_uint32,U8,P,U8,c.c_bool,c.c_bool]
lib.dbt_smp_reconnect_open_role.argtypes=[RP,c.c_uint32,U8,P,U8,c.c_bool,c.c_bool,c.c_bool]
lib.dbt_smp_reconnect_open_candidate.argtypes=[RP,c.c_uint32,U8,P,U8,P,U8,c.c_bool,c.c_bool]
lib.dbt_smp_reconnect_open_candidate_role.argtypes=[RP,c.c_uint32,U8,P,U8,P,U8,c.c_bool,c.c_bool,c.c_bool]
lib.dbt_smp_reconnect_job.argtypes=[RP,c.POINTER(BondJob)]
lib.dbt_smp_reconnect_complete.argtypes=[RP,c.c_uint32,c.c_uint32,c.c_bool,R]
lib.dbt_smp_reconnect_cancel.argtypes=[RP,c.c_uint32,c.c_uint32]
lib.dbt_smp_reconnect_ids.argtypes=[RP,c.POINTER(c.c_uint16),P]
lib.dbt_smp_reconnect_take.argtypes=[RP,c.c_uint16,P,P,c.POINTER(U8)]
lib.dbt_smp_reconnect_encryption.argtypes=[RP,c.c_uint32,c.c_bool]

# Two peers exchange phase 2, submit encryption, then wait for durable storage.
a,b=sc_setup(True)
sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9); sc_transfer(a,b,10); sc_transfer(b,a,11)
x,y=arr(bytes(16)),arr(bytes(16)); xs,ys=U8(),U8()
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_encryption(c.byref(b),202,True)==0
assert a.state==b.state==12 and not a.bond_committed and not b.bond_committed
ja,jb=BondJob(),BondJob()
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0
assert lib.dbt_smp_sc_bond_job(c.byref(b),c.byref(jb))==0
# This test uses an in-memory stand-in for an atomic durable storage provider.
persisted_a=BondRecord.from_buffer_copy(bytes(ja.record))
persisted_b=BondRecord.from_buffer_copy(bytes(jb.record))
assert bytes(persisted_a.ltk)==bytes(x) and bytes(persisted_b.ltk)==bytes(y)
assert lib.dbt_smp_sc_bond_complete(c.byref(a),202,ja.serial,True)==2
assert not a.bond_committed and a.state==12
assert lib.dbt_smp_sc_bond_complete(c.byref(a),201,ja.serial,True)==0
assert lib.dbt_smp_sc_bond_complete(c.byref(b),202,jb.serial,True)==0
assert a.bond_committed and b.bond_committed and a.state==b.state==9

r=Reconnect(); identity=arr(bytes(persisted_a.identity))
assert lib.dbt_smp_reconnect_open(c.byref(r),301,persisted_a.identity_type,identity,7,False,True)==0
job=BondJob(); assert lib.dbt_smp_reconnect_job(c.byref(r),c.byref(job))==0
assert lib.dbt_smp_reconnect_complete(c.byref(r),302,job.serial,True,c.byref(persisted_a))==2
assert lib.dbt_smp_reconnect_complete(c.byref(r),301,job.serial,True,c.byref(persisted_a))==0
assert lib.dbt_smp_reconnect_encryption(c.byref(r),301,True)==2
ediv=c.c_uint16(); rand=arr(bytes(8)); key=arr(bytes(16)); size=U8()
assert lib.dbt_smp_reconnect_ids(c.byref(r),c.byref(ediv),rand)==0
assert ediv.value==0 and bytes(rand)==bytes(8)
assert lib.dbt_smp_reconnect_take(c.byref(r),1,rand,key,c.byref(size))==2
assert lib.dbt_smp_reconnect_take(c.byref(r),0,rand,key,c.byref(size))==0
assert bytes(key)==bytes(x) and size.value==7
assert not r.peer_key_verified
assert lib.dbt_smp_reconnect_encryption(c.byref(r),301,True)==0 and r.state==4
assert r.peer_key_verified
assert lib.dbt_smp_reconnect_encryption(c.byref(r),301,False)==0 and r.state==5 and not r.peer_key_verified
r=Reconnect()
assert lib.dbt_smp_reconnect_open(c.byref(r),302,persisted_a.identity_type,identity,7,True,True)==0
assert lib.dbt_smp_reconnect_job(c.byref(r),c.byref(job))==0
assert lib.dbt_smp_reconnect_complete(c.byref(r),302,job.serial,True,c.byref(persisted_a))==4 and r.state==5

# Store cancellation/failure cannot produce a committed bond.
a,b=sc_setup(True); sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9); sc_transfer(a,b,10); sc_transfer(b,a,11)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0
assert lib.dbt_smp_sc_bond_cancel(c.byref(a),201,ja.serial)==0
assert not a.bond_committed and a.state==10
assert lib.dbt_smp_sc_bond_complete(c.byref(a),201,ja.serial,True)==2
print('SC bond commit gating, simulated reload, reconnect policy and cancellation passed')

# Negotiated signing-key distribution is bound to pairing and storage. A local
# final key cannot be treated as delivered merely because it was submitted.
a,b=sc_setup(True,True)
sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9); sc_transfer(a,b,10); sc_transfer(b,a,11)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_encryption(c.byref(b),202,True)==0
assert a.state==b.state==13 and a.encryption_verified and b.encryption_verified
p=P(); n=U8()
assert lib.dbt_smp_sc_dist_tx(c.byref(b),c.byref(p),c.byref(n),12)==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(b.dist),c.byref(tx_token))==0
data=arr(c.string_at(p,n.value)); assert data[0]==10
assert lib.dbt_smp_sc_dist_receive(c.byref(a),data,len(data),12)==0
assert b.state==13 and lib.dbt_smp_sc_bond_job(c.byref(b),c.byref(jb))==2
assert lib.dbt_smp_sc_dist_tx_admitted(c.byref(b),202,tx_token.value,True,12)==0
assert lib.dbt_smp_sc_dist_tx(c.byref(a),c.byref(p),c.byref(n),13)==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(a.dist),c.byref(tx_token))==0
data=arr(c.string_at(p,n.value)); assert lib.dbt_smp_sc_dist_receive(c.byref(b),data,len(data),13)==0
assert b.state==12 and a.state==13
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==2
assert lib.dbt_smp_sc_dist_tx_admitted(c.byref(a),201,tx_token.value,True,13)==0
assert a.dist.local_pdus_admitted==1 and not a.dist.final_ll_acked
token=c.c_uint32(); assert lib.dbt_smp_dist_final_pending(c.byref(a.dist),c.byref(token))==0
assert lib.dbt_smp_dist_hci_buffers_completed(c.byref(a.dist),201,2)==0
assert a.state==13 and lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==2
assert lib.dbt_smp_sc_dist_final_ack(c.byref(a),201,token.value+1,True,13)==2
assert lib.dbt_smp_sc_dist_final_ack(c.byref(a),202,token.value,True,13)==2
assert lib.dbt_smp_sc_dist_final_ack(c.byref(a),201,token.value,True,13)==0
assert a.state==12 and lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0
assert lib.dbt_smp_sc_bond_job(c.byref(b),c.byref(jb))==0
assert a.final_ll_acked and not a.peer_final_received and a.hci_buffers_completed==2 and a.local_pdus_admitted==1
assert b.peer_final_received and not b.final_ll_acked
assert bytes(ja.record.local_csrk)==bytes([0xA1]*16) and bytes(ja.record.peer_csrk)==bytes([0xB2]*16)
assert bytes(jb.record.local_csrk)==bytes([0xB2]*16) and bytes(jb.record.peer_csrk)==bytes([0xA1]*16)
assert not a.bond_committed and not b.bond_committed
assert lib.dbt_smp_sc_bond_complete(c.byref(a),201,ja.serial,True)==0
assert lib.dbt_smp_sc_bond_complete(c.byref(b),202,jb.serial,True)==0
assert a.bond_committed and b.bond_committed
print('SC negotiated signing-key distribution and durable-store acknowledgment gate passed')

class SCBearer(c.Structure): _fields_=[('engine',SC),('pending',U8*65),('pending_size',U8)]
SBP=c.POINTER(SCBearer)
lib.dbt_smp_sc_bearer_tx.argtypes=[SBP,c.c_uint16,c.c_uint32,c.POINTER(P),c.POINTER(U8),c.c_uint32]
lib.dbt_smp_sc_bearer_tx_admitted.argtypes=[SBP,c.c_uint16,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_bearer_dist_admitted.argtypes=[SBP,c.c_uint16,c.c_uint32,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_bearer_dist_token.argtypes=[SBP,c.c_uint16,c.c_uint32,c.POINTER(c.c_uint32)]
lib.dbt_smp_sc_bearer_dist_ack.argtypes=[SBP,c.c_uint16,c.c_uint32,c.c_uint32,c.c_bool,c.c_uint32]
lib.dbt_smp_sc_bearer_receive.argtypes=[SBP,c.c_uint16,c.c_uint32,c.c_uint16,P,c.c_uint16,c.c_uint32]
lib.dbt_smp_sc_bearer_pump.argtypes=[SBP,c.c_uint16,c.c_uint32,c.c_uint32]
a,b=sc_setup(True,True); sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9); sc_transfer(a,b,10); sc_transfer(b,a,11)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_encryption(c.byref(b),202,True)==0
assert lib.dbt_smp_sc_dist_tx(c.byref(b),c.byref(p),c.byref(n),12)==0
tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(b.dist),c.byref(tx_token))==0
data=arr(c.string_at(p,n.value)); assert lib.dbt_smp_sc_dist_receive(c.byref(a),data,len(data),12)==0
assert lib.dbt_smp_sc_dist_tx_admitted(c.byref(b),202,tx_token.value,True,12)==0
bear=SCBearer(); bear.engine=a
assert lib.dbt_smp_sc_bearer_tx(c.byref(bear),1,201,c.byref(p),c.byref(n),12)==0
tx_token=c.c_uint32(); assert lib.dbt_smp_sc_bearer_dist_token(c.byref(bear),1,201,c.byref(tx_token))==0
assert lib.dbt_smp_sc_bearer_tx_admitted(c.byref(bear),1,201,True,12)==4
assert lib.dbt_smp_sc_bearer_dist_admitted(c.byref(bear),1,201,tx_token.value,True,12)==0
assert bear.engine.dist.final_wait_ack and bear.engine.dist.tx_size==0
token=c.c_uint32(); assert lib.dbt_smp_dist_final_pending(c.byref(bear.engine.dist),c.byref(token))==0
timeout_bear=SCBearer(); timeout_bear.engine=bear.engine
assert lib.dbt_smp_dist_hci_buffers_completed(c.byref(timeout_bear.engine.dist),201,1)==0
lib.dbt_smp_sc_poll(c.byref(timeout_bear.engine),30012)
assert timeout_bear.engine.state==11 and not timeout_bear.engine.bond_committed
assert lib.dbt_smp_sc_bearer_dist_ack(c.byref(timeout_bear),1,201,token.value,True,30012)==5
lost_bear=SCBearer(); lost_bear.engine=bear.engine
assert lost_bear.engine.encryption_verified
assert lib.dbt_smp_sc_encryption(c.byref(lost_bear.engine),201,False)==0
assert lost_bear.engine.state==10 and not lost_bear.engine.encryption_verified and not lost_bear.engine.bond_committed
assert lib.dbt_smp_sc_bearer_dist_ack(c.byref(lost_bear),1,201,token.value,True,13)==5
assert lib.dbt_smp_sc_bearer_dist_ack(c.byref(bear),1,201,token.value,False,12)==0
assert bear.engine.state==10 and not bear.engine.bond_committed
print('SC bearer keeps queue admission separate from phase-3 acknowledgment')

# Numeric Comparison cannot issue DHKey checks until each local user accepts.
a,b=sc_setup(True,False,True); sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9)
assert a.state==b.state==14 and not a.authenticated and not b.authenticated
va,vb=c.c_uint32(),c.c_uint32()
assert lib.dbt_smp_sc_numeric_value(c.byref(a),201,c.byref(va))==0
assert lib.dbt_smp_sc_numeric_value(c.byref(b),202,c.byref(vb))==0
assert va.value==vb.value and va.value<1000000
assert lib.dbt_smp_sc_numeric_confirm(c.byref(a),202,True,10)==2
assert lib.dbt_smp_sc_numeric_confirm(c.byref(a),201,True,10)==0
assert lib.dbt_smp_sc_numeric_confirm(c.byref(b),202,True,10)==0
sc_transfer(a,b,11); sc_transfer(b,a,12)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0 and ja.record.authenticated
assert lib.dbt_smp_sc_bond_complete(c.byref(a),201,ja.serial,True)==0
assert a.state==9 and a.authenticated and a.bond_committed
a,b=sc_setup(False,False,True); sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9)
assert lib.dbt_smp_sc_numeric_confirm(c.byref(a),201,False,10)==0
assert a.state==10 and not a.authenticated and a.tx_data[1]==12
req=arr(bytes([1,3,0,12,16,0,0])); rsp=arr(bytes([2,3,0,12,16,0,0])); z=SC()
assert lib.dbt_smp_sc_open(c.byref(z),1,55,0,0,aa,0,bb,req,rsp,7,1)==4
print('SC Numeric Comparison peer values, user acceptance and authenticated bond policy passed')

# A peer check arriving while user confirmation is pending is retained exactly
# once; another packet receives backpressure. The original input may change.
a,b=sc_setup(False,False,True); sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(a,9); sc_work(b,9)
assert lib.dbt_smp_sc_numeric_confirm(c.byref(a),201,True,10)==0
p=P(); n=U8(); assert lib.dbt_smp_sc_tx(c.byref(a),c.byref(p),c.byref(n))==0
packet=arr(c.string_at(p,n.value)); lib.dbt_smp_sc_tx_done(c.byref(a),True,10)
bear=SCBearer(); bear.engine=b
assert lib.dbt_smp_sc_bearer_receive(c.byref(bear),2,202,6,packet,17,10)==0
assert bear.pending_size==17
assert lib.dbt_smp_sc_bearer_receive(c.byref(bear),2,202,6,packet,17,10)==5
packet[1]^=0xff
assert lib.dbt_smp_sc_bearer_pump(c.byref(bear),2,202,10)==5
assert lib.dbt_smp_sc_numeric_confirm(c.byref(bear.engine),202,True,11)==0
assert lib.dbt_smp_sc_bearer_pump(c.byref(bear),2,202,11)==0
assert bear.pending_size==0 and bear.engine.state==8 and bear.engine.tx_data[0]==13
print('SC bearer retains one blocked packet and applies bounded backpressure')

# Twenty fresh confirm/random rounds, followed by DHKey checks and bond commit.
a,b=sc_setup(True,False,False,(123456,123456))
for round_index in range(20):
    assert a.round==b.round==round_index
    sc_work(a,20+round_index); sc_work(b,20+round_index)
    assert sc_transfer(a,b,20+round_index)==3
    assert sc_transfer(b,a,20+round_index)==3
    assert sc_transfer(a,b,20+round_index)==4
    sc_work(b,20+round_index)
    assert sc_transfer(b,a,20+round_index)==4
    sc_work(a,20+round_index)
assert a.authenticated and b.authenticated and a.job_pending and b.job_pending
sc_work(b,50); sc_work(a,50); sc_transfer(a,b,51); sc_transfer(b,a,52)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0 and bytes(x)==bytes(y)
assert a.passkey==b.passkey==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0 and ja.record.authenticated
a,b=sc_setup(False,False,False,(123456,123457))
sc_work(a,20); sc_work(b,20); sc_transfer(a,b,20); sc_transfer(b,a,20); sc_transfer(a,b,20)
sc_work(b,20)
assert b.state==10 and b.tx_data[1]==4 and b.passkey==0
print('SC Passkey Entry twenty rounds, matching keys and mismatch cleanup passed')

# Bilateral OOB verifies each pre-exchanged commitment, then exchanges nonces.
a,b=sc_setup(True,False,False,None,True)
sc_work(a,5); sc_work(b,5)
assert a.state==b.state==5 and a.authenticated and b.authenticated
assert sc_transfer(a,b,6)==4 and sc_transfer(b,a,7)==4
sc_work(b,8); sc_work(a,8); sc_transfer(a,b,9); sc_transfer(b,a,10)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0 and bytes(x)==bytes(y)
assert bytes(a.local_oob_random)==bytes(16) and bytes(b.peer_oob_random)==bytes(16)
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0 and ja.record.authenticated
a,b=sc_setup(False,False,False,None,True)
a.peer_oob_confirm[0]^=1
# The published worker job is a separate bounded copy; corrupt it as well to
# simulate a bad pre-exchanged commitment without mutating any peer state.
a.job_data.peer_oob_confirm[0]^=1
sc_work(a,5); assert a.state==10 and a.tx_data[1]==4 and not a.authenticated
print('SC bilateral OOB peer commitments, authenticated channel policy and checks passed')

# The intended SC bond cannot silently replace a stronger stored bond.
req=arr(bytes([1,3,0,9,16,0,0])); rsp=arr(bytes([2,3,0,9,7,0,0])); z=SC()
ia=arr(hx('010203040506')); ib=arr(hx('a1a2a3a4a5e6'))
assert lib.dbt_smp_sc_open(c.byref(z),1,81,0,0,ia,1,ib,req,rsp,7,1)==0
assert lib.dbt_smp_sc_check_bond_policy(c.byref(z),True,True,7)==4
assert lib.dbt_smp_sc_check_bond_policy(c.byref(z),True,False,16)==4
assert lib.dbt_smp_sc_check_bond_policy(c.byref(z),True,False,7)==0
assert z.bond_policy_checked
# A late transport admission cannot revive a timed-out job.
sc_work(z,2); assert z.tx_size==65
lib.dbt_smp_sc_tx_done(c.byref(z),True,30002)
assert z.state==11 and z.tx_size==0 and not z.job_pending
print('Bond downgrade policy and late-transport timeout passed')

class FeaturePolicy(c.Structure):
    _fields_=[('io',U8),('minimum',U8),('maximum',U8),('initiator_keys',U8),('responder_keys',U8),
              ('support_sc',c.c_bool),('require_sc',c.c_bool),('bonding',c.c_bool),('mitm',c.c_bool),('oob',c.c_bool)]
class Features(c.Structure):
    _fields_=[('handle',c.c_uint16),('generation',c.c_uint32),('deadline',c.c_uint32),('policy',FeaturePolicy),
              ('state',c.c_int),('preq',U8*7),('pres',U8*7),('tx_data',U8*7),('tx_size',U8),
              ('initiator',c.c_bool),('used',c.c_bool),('sc',c.c_bool),('claimed',c.c_bool),
              ('guard',c.POINTER(Guard)),('result_reported',c.c_bool)]
FP=c.POINTER(Features)
lib.dbt_smp_features_open.argtypes=[FP,c.c_uint16,c.c_uint32,c.c_bool,c.POINTER(FeaturePolicy)]
lib.dbt_smp_features_start.argtypes=[FP,c.c_uint32]
lib.dbt_smp_features_receive.argtypes=[FP,P,U8,c.c_uint32]
lib.dbt_smp_features_tx.argtypes=[FP,c.POINTER(P),c.POINTER(U8)]
lib.dbt_smp_features_tx_done.argtypes=[FP,c.c_bool,c.c_uint32]
lib.dbt_smp_features_take.argtypes=[FP,P,P,c.POINTER(c.c_bool)]
lib.dbt_smp_features_open_sc.argtypes=[FP,SP,U8,P,U8,P,c.c_uint32]
lib.dbt_smp_features_resume_legacy.argtypes=[FP,S,c.c_uint32]
lib.dbt_smp_features_poll.argtypes=[FP,c.c_uint32]
lib.dbt_smp_features_set_guard.argtypes=[FP,G,c.c_uint32]
lib.dbt_smp_features_report_result.argtypes=[FP,c.c_uint32,c.c_bool,c.c_uint32]
pa=FeaturePolicy(1,7,16,4,4,True,True,True,True,False)
pb=FeaturePolicy(1,7,7,0,4,True,True,True,True,False)
fa,fb=Features(),Features()
assert lib.dbt_smp_features_open(c.byref(fa),1,201,True,c.byref(pa))==0
assert lib.dbt_smp_features_open(c.byref(fb),2,202,False,c.byref(pb))==0
ga,gb=Guard(),Guard()
assert lib.dbt_smp_guard_init(c.byref(ga),100,400,1000,0)==0
assert lib.dbt_smp_guard_init(c.byref(gb),100,400,1000,0)==0
assert lib.dbt_smp_features_start(c.byref(fa),1)==2
assert lib.dbt_smp_features_set_guard(c.byref(fa),c.byref(ga),1)==0
assert lib.dbt_smp_features_set_guard(c.byref(fb),c.byref(gb),1)==0
assert lib.dbt_smp_features_start(c.byref(fa),1)==0
p=P(); n=U8(); assert lib.dbt_smp_features_tx(c.byref(fa),c.byref(p),c.byref(n))==0
req=arr(c.string_at(p,n.value)); lib.dbt_smp_features_tx_done(c.byref(fa),True,2)
assert lib.dbt_smp_features_receive(c.byref(fb),req,7,2)==0
assert lib.dbt_smp_features_open_sc(c.byref(fb),c.byref(SC()),1,ib,0,ia,3)==2
assert lib.dbt_smp_features_tx(c.byref(fb),c.byref(p),c.byref(n))==0
rsp=arr(c.string_at(p,n.value)); assert list(rsp)[5:]==[0,4]
lib.dbt_smp_features_tx_done(c.byref(fb),True,3)
assert lib.dbt_smp_features_receive(c.byref(fa),rsp,7,3)==0
fa_sc,fb_sc=SC(),SC()
assert lib.dbt_smp_features_open_sc(c.byref(fa),c.byref(fa_sc),0,ia,1,ib,4)==0
assert lib.dbt_smp_features_open_sc(c.byref(fb),c.byref(fb_sc),1,ib,0,ia,4)==0
assert bytes(fa_sc.preq)==bytes(fb_sc.preq)==bytes(req)
assert bytes(fa_sc.pres)==bytes(fb_sc.pres)==bytes(rsp) and fa_sc.key_size==fb_sc.key_size==7
assert fa_sc.local_mask==fb_sc.peer_mask==0 and fa_sc.peer_mask==fb_sc.local_mask==4
assert lib.dbt_smp_features_report_result(c.byref(fa),201,False,5)==0
assert not lib.dbt_smp_guard_allowed(c.byref(ga),5)
assert lib.dbt_smp_features_open(c.byref(fa),1,201,True,c.byref(pa))==0
ga=Guard(); assert lib.dbt_smp_guard_init(c.byref(ga),100,400,1000,0xfffffff0)==0
assert lib.dbt_smp_features_set_guard(c.byref(fa),c.byref(ga),0xfffffff0)==0
assert lib.dbt_smp_features_start(c.byref(fa),0xfffffff0)==0
lib.dbt_smp_features_poll(c.byref(fa),29984)
assert fa.state==5 and fa.tx_size==0
print('Feature exchange, policy negotiation, transcript binding and timeout passed')

# SC identity keys remain provisional in storage until a reconnect using the
# candidate address and LTK succeeds and a second durable store is acknowledged.
a,b=sc_setup(True,False,False,None,False,True)
sc_transfer(b,a,6); sc_transfer(a,b,7); sc_transfer(b,a,8)
sc_work(b,9); sc_work(a,9); sc_transfer(a,b,10); sc_transfer(b,a,11)
assert lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0
assert lib.dbt_smp_sc_encryption(c.byref(a),201,True)==0
assert lib.dbt_smp_sc_encryption(c.byref(b),202,True)==0
def sc_dist_transfer(src,dst,now):
    p=P(); n=U8(); assert lib.dbt_smp_sc_dist_tx(c.byref(src),c.byref(p),c.byref(n),now)==0
    tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(src.dist),c.byref(tx_token))==0
    data=arr(c.string_at(p,n.value)); assert lib.dbt_smp_sc_dist_receive(c.byref(dst),data,n.value,now)==0
    assert lib.dbt_smp_sc_dist_tx_admitted(c.byref(src),src.generation,tx_token.value,True,now)==0
    token=c.c_uint32()
    if lib.dbt_smp_dist_final_pending(c.byref(src.dist),c.byref(token))==0:
        assert lib.dbt_smp_sc_dist_final_ack(c.byref(src),src.generation,token.value,True,now)==0
    return data[0]
assert [sc_dist_transfer(b,a,12) for _ in range(2)]==[8,9]
assert [sc_dist_transfer(a,b,13) for _ in range(2)]==[8,9]
assert lib.dbt_smp_sc_bond_job(c.byref(a),c.byref(ja))==0
record=BondRecord.from_buffer_copy(bytes(ja.record))
assert record.candidate_present and not record.candidate_verified and record.candidate_type==1
assert bytes(record.candidate_identity)==hx('2122232425c6') and bytes(record.candidate_irk)==bytes([0x24]*16)
assert lib.dbt_smp_sc_bond_complete(c.byref(a),201,ja.serial,True)==0
identity=arr(bytes(record.identity)); candidate=arr(bytes(record.candidate_identity)); r=Reconnect()
assert lib.dbt_smp_reconnect_open_candidate(c.byref(r),401,record.identity_type,identity,record.candidate_type,candidate,7,False,True)==0
assert lib.dbt_smp_reconnect_job(c.byref(r),c.byref(job))==0
assert lib.dbt_smp_reconnect_complete(c.byref(r),401,job.serial,True,c.byref(record))==0
assert lib.dbt_smp_reconnect_ids(c.byref(r),c.byref(ediv),rand)==0
assert lib.dbt_smp_reconnect_take(c.byref(r),ediv.value,rand,key,c.byref(size))==0
assert not r.peer_key_verified and not r.identity_promotion_committed
assert lib.dbt_smp_reconnect_encryption(c.byref(r),401,True)==0 and r.state==6 and r.peer_key_verified
assert not r.identity_promotion_committed
assert not record.candidate_verified
assert lib.dbt_smp_reconnect_job(c.byref(r),c.byref(job))==0
assert job.record.candidate_verified and job.record.candidate_present
assert lib.dbt_smp_reconnect_complete(c.byref(r),401,job.serial,True,None)==0 and r.state==4 and r.identity_promotion_committed
print('SC identity distribution, provisional binding and reconnect promotion acknowledgment passed')

# Legacy peers negotiate and exchange LTK/EDIV/Rand, identity and signing keys.
# Reconnect uses the peripheral's locally distributed LTK and the central's
# corresponding peer LTK, with no security claim until encryption event.
a,b=State(),State(); ia=arr(hx('010203040506')); ib=arr(hx('a1a2a3a4a5e6'))
assert lib.dbt_smp_open(c.byref(a),1,501,0,0,ia,1,ib,7,False)==0
assert lib.dbt_smp_open(c.byref(b),2,502,1,1,ib,0,ia,7,False)==0
assert lib.dbt_smp_set_passkey(c.byref(a),123456,2)==0
assert lib.dbt_smp_set_passkey(c.byref(b),123456,0)==0
assert lib.dbt_smp_set_max_key_size(c.byref(b),7)==0
ka,kb=Keys(),Keys()
c.memmove(ka.ltk,bytes(range(1,17)),16); c.memmove(kb.ltk,bytes(range(21,37)),16)
c.memmove(ka.rand,bytes([1]*8),8); c.memmove(kb.rand,bytes([2]*8),8)
c.memmove(ka.irk,bytes([3]*16),16); c.memmove(kb.irk,bytes([4]*16),16)
c.memmove(ka.csrk,bytes([5]*16),16); c.memmove(kb.csrk,bytes([6]*16),16)
ka.ediv=0x1234; kb.ediv=0x5678; ka.identity_type=0; kb.identity_type=1
c.memmove(ka.identity,hx('111213141516'),6); c.memmove(kb.identity,hx('2122232425c6'),6)
assert lib.dbt_smp_set_bonding(c.byref(a),7,7,c.byref(ka),False,False,False,0)==0
assert lib.dbt_smp_set_bonding(c.byref(b),7,7,c.byref(kb),False,False,False,0)==0
assert lib.dbt_smp_start(c.byref(a),1)==0
transfer(a,b,2); work(b,bytes(range(16)),3); transfer(b,a,4)
work(a,bytes(range(16,32)),5); transfer(a,b,6); transfer(b,a,7); transfer(a,b,8)
work(b,bytes(16),9); transfer(b,a,10); work(a,bytes(16),11)
assert lib.dbt_smp_take_stk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_take_stk(c.byref(b),y,c.byref(ys))==0 and bytes(x)==bytes(y)
assert lib.dbt_smp_encryption(c.byref(a),501,True)==0
assert lib.dbt_smp_encryption(c.byref(b),502,True)==0
assert a.state==b.state==10
def legacy_dist_transfer(src,dst,now):
    p=P(); n=U8(); assert lib.dbt_smp_dist_tx_legacy(c.byref(src),c.byref(p),c.byref(n),now)==0
    tx_token=c.c_uint32(); assert lib.dbt_smp_dist_tx_token(c.byref(src.dist),c.byref(tx_token))==0
    data=arr(c.string_at(p,n.value)); assert lib.dbt_smp_dist_receive_legacy(c.byref(dst),data,n.value,now)==0
    assert lib.dbt_smp_dist_admitted_legacy(c.byref(src),src.generation,tx_token.value,True,now)==0
    token=c.c_uint32()
    if lib.dbt_smp_dist_final_pending(c.byref(src.dist),c.byref(token))==0:
        assert lib.dbt_smp_dist_final_ack_legacy(c.byref(src),src.generation,token.value,True,now)==0
    return data[0]
assert [legacy_dist_transfer(b,a,12) for _ in range(5)]==[6,7,8,9,10]
assert [legacy_dist_transfer(a,b,13) for _ in range(5)]==[6,7,8,9,10]
ja,jb=BondJob(),BondJob()
assert lib.dbt_smp_bond_job_legacy(c.byref(a),c.byref(ja))==0
assert lib.dbt_smp_bond_job_legacy(c.byref(b),c.byref(jb))==0
ra,rb=BondRecord.from_buffer_copy(bytes(ja.record)),BondRecord.from_buffer_copy(bytes(jb.record))
assert ra.have_peer_ltk and ra.have_local_ltk and rb.have_peer_ltk and rb.have_local_ltk
assert bytes(ra.ltk)==bytes(rb.local_ltk)==bytes(kb.ltk)[:7]+bytes(9)
assert ra.ediv==rb.local_ediv==0x5678 and ra.authenticated and not ra.sc
assert ra.candidate_present and not ra.candidate_verified
assert lib.dbt_smp_bond_complete_legacy(c.byref(a),501,ja.serial,True)==0
assert lib.dbt_smp_bond_complete_legacy(c.byref(b),502,jb.serial,True)==0
assert a.bond_committed and b.bond_committed
rc,rp=Reconnect(),Reconnect(); aid=arr(bytes(ra.identity)); bid=arr(bytes(rb.identity))
assert lib.dbt_smp_reconnect_open_role(c.byref(rc),601,ra.identity_type,aid,7,True,False,False)==0
assert lib.dbt_smp_reconnect_open_role(c.byref(rp),602,rb.identity_type,bid,7,True,False,True)==0
assert lib.dbt_smp_reconnect_job(c.byref(rc),c.byref(ja))==0
assert lib.dbt_smp_reconnect_job(c.byref(rp),c.byref(jb))==0
assert lib.dbt_smp_reconnect_complete(c.byref(rc),601,ja.serial,True,c.byref(ra))==0
assert lib.dbt_smp_reconnect_complete(c.byref(rp),602,jb.serial,True,c.byref(rb))==0
ec,ep=c.c_uint16(),c.c_uint16(); nc,np=arr(bytes(8)),arr(bytes(8)); xc,xp=arr(bytes(16)),arr(bytes(16))
assert lib.dbt_smp_reconnect_ids(c.byref(rc),c.byref(ec),nc)==0
assert lib.dbt_smp_reconnect_ids(c.byref(rp),c.byref(ep),np)==0
assert ec.value==ep.value==0x5678 and bytes(nc)==bytes(np)
assert lib.dbt_smp_reconnect_take(c.byref(rc),ec.value,nc,xc,c.byref(xs))==0
assert lib.dbt_smp_reconnect_take(c.byref(rp),ep.value,np,xp,c.byref(ys))==0 and bytes(xc)==bytes(xp)
assert lib.dbt_smp_reconnect_encryption(c.byref(rc),601,True)==0
assert lib.dbt_smp_reconnect_encryption(c.byref(rp),602,True)==0
print('Legacy bond distribution, commit and role-specific simulated reconnect passed')

# Feature negotiation can also hand its exact transcript to the legacy engine.
pa=FeaturePolicy(2,7,16,7,7,False,False,True,True,False)
pb=FeaturePolicy(0,7,7,7,7,False,False,True,True,False)
fa,fb=Features(),Features(); a,b=State(),State()
assert lib.dbt_smp_features_open(c.byref(fa),1,801,True,c.byref(pa))==0
assert lib.dbt_smp_features_open(c.byref(fb),2,802,False,c.byref(pb))==0
ga,gb=Guard(),Guard()
assert lib.dbt_smp_guard_init(c.byref(ga),100,400,1000,0)==0
assert lib.dbt_smp_guard_init(c.byref(gb),100,400,1000,0)==0
assert lib.dbt_smp_features_set_guard(c.byref(fa),c.byref(ga),1)==0
assert lib.dbt_smp_features_set_guard(c.byref(fb),c.byref(gb),1)==0
assert lib.dbt_smp_open(c.byref(a),1,801,0,0,ia,1,ib,7,False)==0
assert lib.dbt_smp_open(c.byref(b),2,802,1,1,ib,0,ia,7,False)==0
assert lib.dbt_smp_set_passkey(c.byref(a),123456,2)==0
assert lib.dbt_smp_set_passkey(c.byref(b),123456,0)==0
assert lib.dbt_smp_set_max_key_size(c.byref(b),7)==0
assert lib.dbt_smp_set_bonding(c.byref(a),7,7,c.byref(ka),False,False,False,0)==0
assert lib.dbt_smp_set_bonding(c.byref(b),7,7,c.byref(kb),False,False,False,0)==0
assert lib.dbt_smp_features_start(c.byref(fa),1)==0
p=P(); n=U8(); assert lib.dbt_smp_features_tx(c.byref(fa),c.byref(p),c.byref(n))==0
req=arr(c.string_at(p,n.value)); lib.dbt_smp_features_tx_done(c.byref(fa),True,2)
assert lib.dbt_smp_features_receive(c.byref(fb),req,7,2)==0
assert lib.dbt_smp_features_tx(c.byref(fb),c.byref(p),c.byref(n))==0
rsp=arr(c.string_at(p,n.value)); lib.dbt_smp_features_tx_done(c.byref(fb),True,3)
assert lib.dbt_smp_features_receive(c.byref(fa),rsp,7,3)==0
assert lib.dbt_smp_features_resume_legacy(c.byref(fa),c.byref(a),4)==0
assert lib.dbt_smp_features_resume_legacy(c.byref(fb),c.byref(b),4)==0
assert lib.dbt_smp_features_resume_legacy(c.byref(fa),c.byref(a),4)==2
assert bytes(a.preq)==bytes(b.preq)==bytes(req) and bytes(a.pres)==bytes(b.pres)==bytes(rsp)
work(a,bytes(range(16,32)),5); work(b,bytes(range(16)),5)
transfer(a,b,6); transfer(b,a,7); transfer(a,b,8); work(b,bytes(16),9)
transfer(b,a,10); work(a,bytes(16),11)
assert lib.dbt_smp_take_stk(c.byref(a),x,c.byref(xs))==0
assert lib.dbt_smp_take_stk(c.byref(b),y,c.byref(ys))==0 and bytes(x)==bytes(y)
print('Legacy feature transcript to phase-2 handoff passed')
