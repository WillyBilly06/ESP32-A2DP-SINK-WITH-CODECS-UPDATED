"""Test the SDK NVS adapter with an in-memory, failure-injectable NVS shim.

This does not exercise flash, NVS recovery or real power loss.
"""
import ctypes as c
import hashlib
import os
import shutil
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
SDK=Path(os.environ.get('DBT_SMP_MBEDTLS_SOURCE',ROOT.parent/'esp-idf/components/mbedtls/mbedtls'))
OUT=ROOT/'_build/smp'
check=[sys.executable,str(ROOT/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    OUT.mkdir(parents=True,exist_ok=True)
    windows=os.name=='nt'
    clang=os.environ.get('CC') or (r'C:\Program Files\LLVM\bin\clang.exe' if windows else shutil.which('cc'))
    if not clang: raise RuntimeError('A C compiler is required')
    dll=OUT/('nvs_mock.dll' if windows else 'nvs_mock.so')
    exports=['dbt_smp_nvs_init','dbt_smp_nvs_execute','dbt_smp_nvs_pending','dbt_smp_nvs_reconcile','dbt_smp_nvs_resolve_rpa','dbt_smp_nvs_verify_rpa',
             'fake_nvs_reset','fake_nvs_fail_commit','fake_nvs_fail_commit_at','fake_nvs_cut_after_commit',
             'fake_nvs_fail_read','fake_nvs_fail_erase','fake_nvs_power_cycle','fake_nvs_corrupt','fake_nvs_corrupt_key',
             'nvs_set_blob','nvs_get_blob','nvs_commit']
    sanflags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('DBT_SMP_SANITIZE')=='1' and not windows else []
    build_env=os.environ.copy()
    if sanflags: build_env.pop('LD_PRELOAD',None)
    platform_flags=['-Wl,/export:'+name for name in exports] if windows else ['-fPIC']
    subprocess.run([clang,'-std=c11','-Wall','-Wextra','-Werror','-shared',*sanflags,'-I',str(ROOT/'tests/smp'),
        '-I',str(ROOT/'include'),'-I',str(SDK/'include'),'-I',str(SDK/'library'),
        str(ROOT/'src/esp32/dbt_smp_nvs.c'),str(ROOT/'src/dbt_smp_crypto.c'),str(ROOT/'tests/smp/nvs_mock.c'),
        str(SDK/'library/aes.c'),str(SDK/'library/aesni.c'),str(SDK/'library/sha256.c'),str(SDK/'library/platform_util.c'),
        *platform_flags,'-o',str(dll)],check=True,env=build_env)
    os.environ['DBT_SMP_TEST_BINARY']='smp_nvs_core.dll'
    import test_smp as t
    lib=c.CDLL(str(dll))
    class Provider(c.Structure): _fields_=[('handle',c.c_uint32),('poisoned',c.c_bool)]
    lib.dbt_smp_nvs_init.argtypes=[c.POINTER(Provider),c.c_uint32]
    lib.dbt_smp_nvs_execute.argtypes=[c.POINTER(Provider),c.POINTER(t.BondJob),c.POINTER(t.BondRecord)]
    lib.dbt_smp_nvs_pending.argtypes=[c.POINTER(Provider),c.POINTER(c.c_int),c.POINTER(t.U8),t.P]
    lib.dbt_smp_nvs_reconcile.argtypes=[c.POINTER(Provider),c.c_int,t.U8,t.P,c.POINTER(c.c_bool)]
    lib.dbt_smp_nvs_resolve_rpa.argtypes=[c.POINTER(Provider),t.P,c.POINTER(t.BondRecord),c.POINTER(c.c_bool)]
    lib.dbt_smp_nvs_verify_rpa.argtypes=[t.P,t.P,c.POINTER(c.c_bool)]
    lib.dbt_smp_nvs_verify_rpa.restype=c.c_bool
    lib.fake_nvs_fail_commit.argtypes=[c.c_bool]
    lib.fake_nvs_fail_commit_at.argtypes=[c.c_uint,c.c_bool]
    lib.fake_nvs_cut_after_commit.argtypes=[c.c_uint]
    lib.fake_nvs_fail_read.argtypes=[c.c_char_p,c.c_uint]
    lib.fake_nvs_fail_erase.argtypes=[c.c_char_p,c.c_uint]
    lib.fake_nvs_corrupt_key.argtypes=[c.c_char_p,c.c_uint]
    lib.nvs_set_blob.argtypes=[c.c_uint32,c.c_char_p,c.c_void_p,c.c_size_t]
    lib.nvs_get_blob.argtypes=[c.c_uint32,c.c_char_p,c.c_void_p,c.POINTER(c.c_size_t)]
    p=Provider(); lib.fake_nvs_reset(); lib.dbt_smp_nvs_init(c.byref(p),1)
    j=t.BondJob(); j.generation=1; j.serial=1; j.identity_type=0
    c.memmove(j.identity,t.hx('010203040506'),6)
    rec=t.BondRecord(); rec.identity_type=0; c.memmove(rec.identity,j.identity,6)
    rec.key_size=16; rec.sc=True; rec.authenticated=True
    c.memmove(rec.ltk,bytes(range(16)),16); j.record=rec
    out=t.BondRecord(); j.op=2
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==1
    j.op=1; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==0
    j.op=2; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==0
    assert bytes(out)==bytes(rec)
    j.op=1; weak=t.BondRecord.from_buffer_copy(bytes(rec)); weak.authenticated=False; j.record=weak
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==4 and not p.poisoned
    stronger=t.BondRecord.from_buffer_copy(bytes(rec)); stronger.ltk[0]^=1; j.record=stronger
    lib.fake_nvs_fail_commit(True)
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==2 and p.poisoned
    j.op=2; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==2
    lib.fake_nvs_power_cycle(); lib.dbt_smp_nvs_init(c.byref(p),1)
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==0 and bytes(out)==bytes(rec)
    j.op=3; lib.fake_nvs_fail_commit(True)
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==2 and p.poisoned
    lib.fake_nvs_power_cycle(); lib.dbt_smp_nvs_init(c.byref(p),1); j.op=2
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==0
    j.op=3; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==0
    j.op=2; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==1
    j.op=1; j.record=rec; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==0
    lib.fake_nvs_corrupt(); j.op=2
    assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==2
    j.op=1; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==2
    lib.fake_nvs_reset(); lib.dbt_smp_nvs_init(c.byref(p),1)
    legacy=t.BondRecord(); legacy.identity_type=0; c.memmove(legacy.identity,j.identity,6)
    legacy.key_size=7; legacy.have_peer_ltk=True; legacy.have_local_ltk=True
    c.memmove(legacy.ltk,bytes(range(1,8))+bytes(9),16)
    c.memmove(legacy.local_ltk,bytes(range(21,28))+bytes(9),16)
    c.memmove(legacy.local_rand,bytes([8]*8),8); legacy.local_ediv=0x1234
    legacy.candidate_present=True; legacy.candidate_verified=True; legacy.candidate_type=1
    c.memmove(legacy.candidate_identity,t.hx('2122232425c6'),6)
    c.memmove(legacy.candidate_irk,bytes([9]*16),16)
    j.record=legacy; j.op=1; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==0
    j.op=2; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),c.byref(out))==0 and bytes(out)==bytes(legacy)
    weaker=t.BondRecord.from_buffer_copy(bytes(legacy)); weaker.candidate_verified=False
    j.record=weaker; j.op=1; assert lib.dbt_smp_nvs_execute(c.byref(p),c.byref(j),None)==4

    # Every cut point reconstructs the provider while retaining committed keys.
    # The two fixtures are synthetic; no secret bytes leave the process.
    identity=t.arr(t.hx('010203040506')); other_identity=t.arr(t.hx('111213141516'))
    old=t.BondRecord(); old.identity_type=0; c.memmove(old.identity,identity,6)
    old.key_size=16; old.sc=True; old.authenticated=True; c.memmove(old.ltk,bytes(range(16)),16)
    new=t.BondRecord.from_buffer_copy(bytes(old)); new.ltk[0]^=0x80
    unrelated=t.BondRecord.from_buffer_copy(bytes(old)); c.memmove(unrelated.identity,other_identity,6)
    unrelated.ltk[0]^=0x40
    def job(op,record=None,which=identity):
        z=t.BondJob(); z.generation=42; z.serial=3; z.op=op; z.identity_type=0
        c.memmove(z.identity,which,6)
        if record is not None: z.record=record
        return z
    def fresh():
        provider=Provider(); lib.dbt_smp_nvs_init(c.byref(provider),1); return provider
    def execute(provider,request,out=None): return lib.dbt_smp_nvs_execute(c.byref(provider),c.byref(request),out)
    def load(provider,which=identity):
        record=t.BondRecord(); rc=execute(provider,job(2,which=which),c.byref(record)); return rc,record
    def pending(provider):
        op=c.c_int(-1); typ=t.U8(255); addr=t.arr(bytes(6))
        rc=lib.dbt_smp_nvs_pending(c.byref(provider),c.byref(op),c.byref(typ),addr)
        return rc,op.value,typ.value,bytes(addr)
    def reconcile(provider,op,which=identity):
        applied=c.c_bool(False)
        rc=lib.dbt_smp_nvs_reconcile(c.byref(provider),op,0,which,c.byref(applied))
        return rc,applied.value
    def setup():
        lib.fake_nvs_reset(); provider=fresh()
        assert execute(provider,job(1,old))==0 and execute(provider,job(1,unrelated,other_identity))==0
        return provider
    for operation in (1,3):
        for stage in (1,2,3):
            p=setup(); lib.fake_nvs_cut_after_commit(stage)
            assert execute(p,job(operation,new if operation==1 else None))==2 and p.poisoned
            lib.fake_nvs_power_cycle(); p=fresh()
            if stage<3:
                assert pending(p)==(0,operation,0,bytes(identity))
                assert load(p)[0]==5 and load(p,other_identity)[0]==5
                blocked=t.BondRecord(); c.memset(c.byref(blocked),0xa5,c.sizeof(blocked))
                assert execute(p,job(2),c.byref(blocked))==5 and bytes(blocked)==bytes(c.sizeof(blocked))
                assert reconcile(p,operation,other_identity)[0]==2
                assert reconcile(p,3 if operation==1 else 1)[0]==2
                rc,applied=reconcile(p,operation); assert rc==0 and applied==(stage==2)
            else: assert pending(p)[0]==1
            rc,observed=load(p)
            if operation==3 and stage>=2: assert rc==1
            else: assert rc==0 and bytes(observed)==bytes(new if operation==1 and stage>=2 else old)
            rc,observed=load(p,other_identity); assert rc==0 and bytes(observed)==bytes(unrelated)
    # Every commit can fail before or after its mutations became durable.
    for stage in (1,2,3):
        for applied_at_failure in (False,True):
            p=setup(); lib.fake_nvs_fail_commit_at(stage,applied_at_failure)
            assert execute(p,job(1,new))==2 and p.poisoned
            lib.fake_nvs_power_cycle(); p=fresh(); rc,_,_,_=pending(p)
            if rc==0:
                rc,applied=reconcile(p,1); assert rc==0 and applied==(stage>=3 or (stage==2 and applied_at_failure))
            else: assert rc==1
            rc,observed=load(p); assert rc==0
            assert bytes(observed)==bytes(new if stage>=3 or (stage==2 and applied_at_failure) else old)
            assert bytes(load(p,other_identity)[1])==bytes(unrelated)
    # Independently check the exact persisted intent digests against hashlib.
    p=setup(); old_raw=(t.U8*154)(); length=c.c_size_t(154)
    assert lib.nvs_get_blob(1,b'0010203040506',old_raw,c.byref(length))==0
    lib.fake_nvs_cut_after_commit(2); assert execute(p,job(1,new))==2
    lib.fake_nvs_power_cycle(); marker=(t.U8*110)(); length=c.c_size_t(110)
    assert lib.nvs_get_blob(1,b'smp_txn_v1',marker,c.byref(length))==0 and length.value==110
    new_raw=(t.U8*154)(); length=c.c_size_t(154)
    assert lib.nvs_get_blob(1,b'0010203040506',new_raw,c.byref(length))==0
    raw_marker=bytes(marker)
    assert raw_marker[:4]==b'DBTI' and raw_marker[4]==1 and raw_marker[13]==1
    assert raw_marker[14:46]==hashlib.sha256(bytes(old_raw)).digest()
    assert raw_marker[46:78]==hashlib.sha256(bytes(new_raw)).digest()
    assert raw_marker[78:110]==hashlib.sha256(raw_marker[:78]).digest()
    # Failure at each readback or marker deletion retains an actionable intent.
    for fault,key,call,expected_applied in [('read',b'smp_txn_v1',2,False),
                                            ('read',b'0010203040506',2,True),
                                            ('erase',b'smp_txn_v1',1,True)]:
        p=setup()
        (lib.fake_nvs_fail_read if fault=='read' else lib.fake_nvs_fail_erase)(key,call)
        assert execute(p,job(1,new))==2 and p.poisoned
        lib.fake_nvs_power_cycle(); p=fresh(); assert pending(p)[0]==0
        assert reconcile(p,1)==(0,expected_applied)
        assert bytes(load(p,other_identity)[1])==bytes(unrelated)
    # If the final marker-removal verification fails after its commit, the
    # verified target may remain while no intent is present after reboot.
    p=setup(); lib.fake_nvs_fail_read(b'smp_txn_v1',3)
    assert execute(p,job(1,new))==2 and p.poisoned
    lib.fake_nvs_power_cycle(); p=fresh(); assert pending(p)[0]==1
    assert bytes(load(p)[1])==bytes(new) and bytes(load(p,other_identity)[1])==bytes(unrelated)
    # A corrupt or unsupported marker is never discarded or treated as absent.
    for offset in (0,4,5,6,13,78):
        p=setup(); lib.fake_nvs_cut_after_commit(1); assert execute(p,job(1,new))==2
        lib.fake_nvs_power_cycle(); lib.fake_nvs_corrupt_key(b'smp_txn_v1',offset); p=fresh()
        assert pending(p)[0]==2 and p.poisoned
        p=fresh(); assert reconcile(p,1)[0]==2 and p.poisoned
    for malformed in (b'DBTI',bytes(111)):
        lib.fake_nvs_reset(); blob=(t.U8*len(malformed)).from_buffer_copy(malformed)
        assert lib.nvs_set_blob(1,b'smp_txn_v1',blob,len(blob))==0 and lib.nvs_commit(1)==0
        p=fresh(); assert pending(p)[0]==2 and p.poisoned
    # A valid-looking third state matches neither intent digest and is untouched.
    p=setup(); lib.fake_nvs_cut_after_commit(2); assert execute(p,job(1,new))==2
    lib.fake_nvs_power_cycle(); raw=(t.U8*154)(); length=c.c_size_t(154)
    assert lib.nvs_get_blob(1,b'0010203040506',raw,c.byref(length))==0 and length.value==154
    raw[12]^=0x22; assert lib.nvs_set_blob(1,b'0010203040506',raw,154)==0 and lib.nvs_commit(1)==0
    p=fresh(); assert reconcile(p,1)[0]==5 and pending(p)[0]==0
    raw_check=(t.U8*154)(); length=c.c_size_t(154)
    assert lib.nvs_get_blob(1,b'0010203040506',raw_check,c.byref(length))==0 and bytes(raw_check)==bytes(raw)
    # Reconciliation cleanup failure preserves intent or leaves only a verified
    # state when its cleanup commit happened despite an error.
    for committed in (False,True):
        p=setup(); lib.fake_nvs_cut_after_commit(2); assert execute(p,job(1,new))==2
        lib.fake_nvs_power_cycle(); p=fresh(); lib.fake_nvs_fail_commit_at(1,committed)
        assert reconcile(p,1)[0]==2 and p.poisoned
        lib.fake_nvs_power_cycle(); p=fresh()
        if committed: assert pending(p)[0]==1
        else: assert pending(p)[0]==0 and reconcile(p,1)==(0,True)
        assert bytes(load(p)[1])==bytes(new)
    p=setup(); lib.fake_nvs_cut_after_commit(2); assert execute(p,job(1,new))==2
    lib.fake_nvs_power_cycle(); p=fresh(); lib.fake_nvs_fail_read(b'smp_txn_v1',2)
    assert reconcile(p,1)[0]==2 and p.poisoned
    lib.fake_nvs_power_cycle(); p=fresh(); assert pending(p)[0]==1 and bytes(load(p)[1])==bytes(new)
    # A version-2 bond written directly, with no marker, remains readable.
    lib.fake_nvs_reset(); raw=(t.U8*154)()
    c.memmove(raw,b'DBTB',4); raw[4]=2; raw[5]=0; c.memmove(c.byref(raw,6),identity,6)
    for k in range(16): raw[12+k]=k
    raw[54]=16; raw[55]=3
    assert lib.nvs_set_blob(1,b'0010203040506',raw,154)==0 and lib.nvs_commit(1)==0
    p=fresh(); rc,observed=load(p); assert rc==0 and bytes(observed)==bytes(old)
    # Bluetooth ah vector and bounded IRK lookup, including collision denial.
    irk=t.le('ec0234a357c8ad05341010a60a397d9b'); rpa=t.le('7081940dfbaa')
    checked=c.c_bool(False); assert lib.dbt_smp_nvs_verify_rpa(rpa,irk,c.byref(checked)) and checked.value
    assert lib.dbt_smp_nvs_verify_rpa(t.le('7081940dfbab'),irk,c.byref(checked)) and not checked.value
    irk_record=t.BondRecord.from_buffer_copy(bytes(old)); irk_record.candidate_present=True
    irk_record.candidate_type=0; c.memmove(irk_record.candidate_identity,identity,6)
    c.memmove(irk_record.candidate_irk,irk,16)
    lib.fake_nvs_reset(); p=fresh(); assert execute(p,job(1,irk_record))==0
    changed_irk=t.BondRecord.from_buffer_copy(bytes(irk_record)); changed_irk.candidate_irk[0]^=1
    assert execute(p,job(1,changed_irk))==4
    removed_identity=t.BondRecord.from_buffer_copy(bytes(irk_record)); removed_identity.candidate_present=False
    assert execute(p,job(1,removed_identity))==4
    promoted=t.BondRecord.from_buffer_copy(bytes(irk_record)); promoted.have_irk=True
    c.memmove(promoted.irk,irk,16); promoted.candidate_verified=True
    assert execute(p,job(1,promoted))==0
    inconsistent=t.BondRecord.from_buffer_copy(bytes(promoted)); inconsistent.candidate_irk[0]^=1
    assert execute(p,job(1,inconsistent))==2
    candidate=c.c_bool(False); resolved=t.BondRecord()
    assert lib.dbt_smp_nvs_resolve_rpa(c.byref(p),rpa,c.byref(resolved),c.byref(candidate))==0
    assert candidate.value and bytes(resolved)==bytes(promoted)
    wrong=t.le('7081940dfbab'); assert lib.dbt_smp_nvs_resolve_rpa(c.byref(p),wrong,c.byref(resolved),c.byref(candidate))==1
    collision=t.BondRecord.from_buffer_copy(bytes(irk_record)); c.memmove(collision.identity,other_identity,6)
    c.memmove(collision.candidate_identity,other_identity,6)
    assert execute(p,job(1,collision,other_identity))==0
    assert lib.dbt_smp_nvs_resolve_rpa(c.byref(p),rpa,c.byref(resolved),c.byref(candidate))==2
    lib.fake_nvs_reset(); p=fresh(); assert execute(p,job(1,old))==0
    assert execute(p,job(1,irk_record))==4  # no pre-existing IRK anchors identity upgrade
    lib.fake_nvs_reset(); p=fresh(); assert execute(p,job(1,irk_record))==0
    one=(t.U8*1)(1)
    for i in range(33): assert lib.nvs_set_blob(1,f'z{i:02}'.encode(),one,1)==0 and lib.nvs_commit(1)==0
    assert lib.dbt_smp_nvs_resolve_rpa(c.byref(p),rpa,c.byref(resolved),c.byref(candidate))==3
    print('NVS versioned intent, before/after reconciliation, cut-stage and failure model tests passed')
finally:
    subprocess.run(check,check=True)
