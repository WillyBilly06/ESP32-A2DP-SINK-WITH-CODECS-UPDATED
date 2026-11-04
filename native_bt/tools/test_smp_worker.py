"""Exercise the SDK mbedTLS SMP worker against paired portable SC engines.

Builds the SDK mbedTLS sources for the host inside the fixed SMP build
directory. This is host evidence, not an ESP32 link or hardware test.
"""
import ctypes as c
import os
import shutil
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parent
OUT = ROOT/'_build/smp'
windows=os.name=='nt'
sanitized=os.environ.get('DBT_SMP_SANITIZE')=='1' and not windows
BUILD = OUT/('mbedtls-cl' if windows else 'mbedtls-linux-sanitized' if sanitized else 'mbedtls-linux')
SDK = Path(os.environ.get('DBT_SMP_MBEDTLS_SOURCE', PROJECT/'esp-idf/components/mbedtls/mbedtls'))
cc = os.environ.get('CC') or (r'C:\Program Files\LLVM\bin\clang.exe' if windows else shutil.which('cc'))
if not cc: raise RuntimeError('A C compiler is required')
clang = Path(cc)
ninja = os.environ.get('NINJA') or (r'C:\Users\ngocv\.espressif\tools\ninja\1.12.1\ninja.exe' if windows else shutil.which('ninja'))
if not ninja: raise RuntimeError('Ninja is required')
sanflags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if sanitized else []
build_env=os.environ.copy()
if sanitized: build_env.pop('LD_PRELOAD',None)
check = [sys.executable, str(ROOT/'tools/check_workspace.py')]
subprocess.run(check, check=True)
try:
    platform_cmake=([f'-DCMAKE_C_COMPILER={clang.with_name("clang-cl.exe")}',
        f'-DCMAKE_RC_COMPILER={clang.with_name("llvm-rc.exe")}',f'-DCMAKE_MT={clang.with_name("llvm-mt.exe")}']
        if windows else [f'-DCMAKE_C_COMPILER={cc}','-DCMAKE_POSITION_INDEPENDENT_CODE=ON'])
    if sanitized: platform_cmake+=['-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer']
    subprocess.run(['cmake','-S',str(SDK),'-B',str(BUILD),'-G','Ninja',
        f'-DCMAKE_MAKE_PROGRAM={ninja}',*platform_cmake, '-DCMAKE_BUILD_TYPE=Release',
        '-DENABLE_TESTING=OFF','-DENABLE_PROGRAMS=OFF',
        '-DUSE_STATIC_MBEDTLS_LIBRARY=ON','-DUSE_SHARED_MBEDTLS_LIBRARY=OFF'],check=True,env=build_env)
    subprocess.run(['cmake','--build',str(BUILD),'--target','mbedcrypto','-j','4'],check=True,env=build_env)
    exports=['open','close','oob','legacy','sc','distribution']
    dll=OUT/('worker_host.dll' if windows else 'worker_host.so')
    importlib=BUILD/('library/mbedcrypto.lib' if windows else 'library/libmbedcrypto.a')
    extras=([str(BUILD/'3rdparty/everest/everest.lib'),str(BUILD/'3rdparty/p256-m/p256m.lib')]
        if windows else [str(BUILD/'3rdparty/everest/libeverest.a'),str(BUILD/'3rdparty/p256-m/libp256m.a'),'-pthread'])
    platform_flags=['-Wl,/export:dbt_smp_worker_mbedtls_'+name for name in exports] if windows else ['-fPIC']
    subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror','-shared',*sanflags,
        '-I',str(ROOT/'include'),'-I',str(SDK/'include'),str(ROOT/'src/esp32/dbt_smp_worker_mbedtls.c'),
        str(ROOT/'src/dbt_smp_crypto.c'),str(importlib),*extras,*platform_flags,'-o',str(dll)],check=True,env=build_env)
    os.environ['DBT_SMP_TEST_BINARY']='smp_worker_core.dll'
    import test_smp as t
    lib=c.CDLL(str(dll)); P=t.P
    class Worker(c.Structure): _fields_=[('internal',c.c_void_p)]
    W=c.POINTER(Worker)
    RNG=c.CFUNCTYPE(c.c_bool,c.c_void_p,P,c.c_size_t)
    @RNG
    def random_bytes(_,out,n):
        try: c.memmove(out,os.urandom(n),n); return True
        except OSError: return False
    lib.dbt_smp_worker_mbedtls_open.argtypes=[W,c.c_uint32,RNG,c.c_void_p]
    lib.dbt_smp_worker_mbedtls_open.restype=c.c_bool
    lib.dbt_smp_worker_mbedtls_close.argtypes=[W]
    lib.dbt_smp_worker_mbedtls_oob.argtypes=[W,P,P,P]
    lib.dbt_smp_worker_mbedtls_oob.restype=c.c_bool
    lib.dbt_smp_worker_mbedtls_sc.argtypes=[W,c.POINTER(t.SCJob),c.POINTER(t.SCResult)]
    lib.dbt_smp_worker_mbedtls_sc.restype=c.c_bool
    lib.dbt_smp_worker_mbedtls_legacy.argtypes=[W,c.POINTER(t.Job),c.POINTER(t.Result)]
    lib.dbt_smp_worker_mbedtls_legacy.restype=c.c_bool
    lib.dbt_smp_worker_mbedtls_distribution.argtypes=[W,t.U8,t.U8,P,c.POINTER(t.Keys)]
    lib.dbt_smp_worker_mbedtls_distribution.restype=c.c_bool

    def work(state,worker,now):
        job=t.SCJob(); result=t.SCResult()
        assert t.lib.dbt_smp_sc_job(c.byref(state),c.byref(job))==0
        assert lib.dbt_smp_worker_mbedtls_sc(c.byref(worker),c.byref(job),c.byref(result))
        replay=t.SCResult()
        assert not lib.dbt_smp_worker_mbedtls_sc(c.byref(worker),c.byref(job),c.byref(replay))
        assert t.lib.dbt_smp_sc_complete(c.byref(state),job.generation,job.serial,True,c.byref(result),now)==0
    def pair(mode):
        wa,wb=Worker(),Worker()
        assert lib.dbt_smp_worker_mbedtls_open(c.byref(wa),701,random_bytes,None)
        assert lib.dbt_smp_worker_mbedtls_open(c.byref(wb),702,random_bytes,None)
        try:
            oob=mode=='oob'; numeric=mode=='numeric'; passkey=mode=='passkey'
            flags=8|(4 if numeric or passkey or oob else 0)
            ia=t.arr(t.hx('010203040506')); ib=t.arr(t.hx('a1a2a3a4a5e6'))
            req=t.arr(bytes([1,1 if numeric else 2 if passkey else 3,int(oob),flags,16,0,0]))
            rsp=t.arr(bytes([2,1 if numeric else 0 if passkey else 3,int(oob),flags,16,0,0]))
            a,b=t.SC(),t.SC()
            assert t.lib.dbt_smp_sc_open(c.byref(a),1,701,0,0,ia,1,ib,req,rsp,7,1)==0
            assert t.lib.dbt_smp_sc_open(c.byref(b),2,702,1,1,ib,0,ia,req,rsp,7,1)==0
            if passkey:
                assert t.lib.dbt_smp_sc_set_passkey(c.byref(a),456789)==0
                assert t.lib.dbt_smp_sc_set_passkey(c.byref(b),456789)==0
            if oob:
                ap,bp=t.arr(bytes(64)),t.arr(bytes(64)); ar,br=t.arr(bytes(16)),t.arr(bytes(16)); ac,bc=t.arr(bytes(16)),t.arr(bytes(16))
                assert lib.dbt_smp_worker_mbedtls_oob(c.byref(wa),ap,ar,ac)
                assert lib.dbt_smp_worker_mbedtls_oob(c.byref(wb),bp,br,bc)
                assert t.lib.dbt_smp_sc_set_oob(c.byref(a),ap,ar,br,bc,True)==0
                assert t.lib.dbt_smp_sc_set_oob(c.byref(b),bp,br,ar,ac,True)==0
            work(a,wa,2); work(b,wb,2)
            t.sc_transfer(a,b,3); t.sc_transfer(b,a,4)
            if passkey:
                for i in range(20):
                    work(a,wa,5+i); work(b,wb,5+i)
                    t.sc_transfer(a,b,5+i); t.sc_transfer(b,a,5+i); t.sc_transfer(a,b,5+i)
                    work(b,wb,5+i); t.sc_transfer(b,a,5+i); work(a,wa,5+i)
            elif oob:
                work(a,wa,5); work(b,wb,5); t.sc_transfer(a,b,6); t.sc_transfer(b,a,7)
            else:
                work(b,wb,5); t.sc_transfer(b,a,6); t.sc_transfer(a,b,7); t.sc_transfer(b,a,8)
            work(b,wb,40); work(a,wa,40)
            if numeric:
                av,bv=c.c_uint32(),c.c_uint32()
                assert t.lib.dbt_smp_sc_numeric_value(c.byref(a),701,c.byref(av))==0
                assert t.lib.dbt_smp_sc_numeric_value(c.byref(b),702,c.byref(bv))==0 and av.value==bv.value
                assert t.lib.dbt_smp_sc_numeric_confirm(c.byref(a),701,True,41)==0
                assert t.lib.dbt_smp_sc_numeric_confirm(c.byref(b),702,True,41)==0
            t.sc_transfer(a,b,42); t.sc_transfer(b,a,43)
            x,y=t.arr(bytes(16)),t.arr(bytes(16)); xs,ys=t.U8(),t.U8()
            assert t.lib.dbt_smp_sc_take_ltk(c.byref(a),x,c.byref(xs))==0
            assert t.lib.dbt_smp_sc_take_ltk(c.byref(b),y,c.byref(ys))==0
            assert bytes(x)==bytes(y) and xs.value==ys.value==16
        finally:
            lib.dbt_smp_worker_mbedtls_close(c.byref(wa)); lib.dbt_smp_worker_mbedtls_close(c.byref(wb))
    for mode in ('justworks','numeric','passkey','oob'): pair(mode)
    def legacy_pair():
        wa,wb=Worker(),Worker()
        assert lib.dbt_smp_worker_mbedtls_open(c.byref(wa),801,random_bytes,None)
        assert lib.dbt_smp_worker_mbedtls_open(c.byref(wb),802,random_bytes,None)
        try:
            a,b=t.State(),t.State(); ia=t.arr(t.hx('010203040506')); ib=t.arr(t.hx('a1a2a3a4a5e6'))
            assert t.lib.dbt_smp_open(c.byref(a),1,801,0,0,ia,1,ib,7,False)==0
            assert t.lib.dbt_smp_open(c.byref(b),2,802,1,1,ib,0,ia,7,False)==0
            assert t.lib.dbt_smp_set_passkey(c.byref(a),456789,2)==0
            assert t.lib.dbt_smp_set_passkey(c.byref(b),456789,0)==0
            ka,kb=t.Keys(),t.Keys()
            assert lib.dbt_smp_worker_mbedtls_distribution(c.byref(wa),7,0,ia,c.byref(ka))
            assert lib.dbt_smp_worker_mbedtls_distribution(c.byref(wb),7,1,ib,c.byref(kb))
            assert not lib.dbt_smp_worker_mbedtls_distribution(c.byref(wb),7,1,ib,c.byref(t.Keys()))
            assert t.lib.dbt_smp_set_bonding(c.byref(a),7,7,c.byref(ka),False,False,False,0)==0
            assert t.lib.dbt_smp_set_bonding(c.byref(b),7,7,c.byref(kb),False,False,False,0)==0
            def lw(state,worker,now):
                job=t.Job(); result=t.Result()
                assert t.lib.dbt_smp_job(c.byref(state),c.byref(job))==0
                assert lib.dbt_smp_worker_mbedtls_legacy(c.byref(worker),c.byref(job),c.byref(result))
                assert t.lib.dbt_smp_complete(c.byref(state),job.generation,job.serial,True,c.byref(result),now)==0
            assert t.lib.dbt_smp_start(c.byref(a),1)==0
            t.transfer(a,b,2); lw(b,wb,3); t.transfer(b,a,4); lw(a,wa,5)
            t.transfer(a,b,6); t.transfer(b,a,7); t.transfer(a,b,8); lw(b,wb,9)
            t.transfer(b,a,10); lw(a,wa,11)
            x,y=t.arr(bytes(16)),t.arr(bytes(16)); xs,ys=t.U8(),t.U8()
            assert t.lib.dbt_smp_take_stk(c.byref(a),x,c.byref(xs))==0
            assert t.lib.dbt_smp_take_stk(c.byref(b),y,c.byref(ys))==0 and bytes(x)==bytes(y)
            assert t.lib.dbt_smp_encryption(c.byref(a),801,True)==0
            assert t.lib.dbt_smp_encryption(c.byref(b),802,True)==0
            for source,target in ((b,a),(a,b)):
                for _ in range(5):
                    p=t.P(); n=t.U8()
                    assert t.lib.dbt_smp_dist_tx_legacy(c.byref(source),c.byref(p),c.byref(n),12)==0
                    tx_token=c.c_uint32(); assert t.lib.dbt_smp_dist_tx_token(c.byref(source.dist),c.byref(tx_token))==0
                    packet=t.arr(c.string_at(p,n.value))
                    assert t.lib.dbt_smp_dist_receive_legacy(c.byref(target),packet,n.value,12)==0
                    assert t.lib.dbt_smp_dist_admitted_legacy(c.byref(source),source.generation,tx_token.value,True,12)==0
                    token=c.c_uint32()
                    if t.lib.dbt_smp_dist_final_pending(c.byref(source.dist),c.byref(token))==0:
                        assert t.lib.dbt_smp_dist_final_ack_legacy(c.byref(source),source.generation,token.value,True,12)==0
            ja,jb=t.BondJob(),t.BondJob()
            assert t.lib.dbt_smp_bond_job_legacy(c.byref(a),c.byref(ja))==0
            assert t.lib.dbt_smp_bond_job_legacy(c.byref(b),c.byref(jb))==0
            assert bytes(ja.record.ltk)==bytes(jb.record.local_ltk)==bytes(kb.ltk)
        finally:
            lib.dbt_smp_worker_mbedtls_close(c.byref(wa)); lib.dbt_smp_worker_mbedtls_close(c.byref(wb))
    legacy_pair()
    def reject_peer(peer_be):
        wa,wb=Worker(),Worker()
        assert lib.dbt_smp_worker_mbedtls_open(c.byref(wa),901,random_bytes,None)
        assert lib.dbt_smp_worker_mbedtls_open(c.byref(wb),902,random_bytes,None)
        try:
            a,b=t.SC(),t.SC(); ia=t.arr(t.hx('010203040506')); ib=t.arr(t.hx('a1a2a3a4a5e6'))
            req=t.arr(bytes([1,3,0,8,16,0,0])); rsp=t.arr(bytes([2,3,0,8,16,0,0]))
            assert t.lib.dbt_smp_sc_open(c.byref(a),1,901,0,0,ia,1,ib,req,rsp,7,1)==0
            assert t.lib.dbt_smp_sc_open(c.byref(b),2,902,1,1,ib,0,ia,req,rsp,7,1)==0
            work(a,wa,2); work(b,wb,2); t.sc_transfer(a,b,3); t.sc_transfer(b,a,4)
            work(b,wb,5); t.sc_transfer(b,a,6); t.sc_transfer(a,b,7); t.sc_transfer(b,a,8)
            j=t.SCJob(); r=t.SCResult(); assert t.lib.dbt_smp_sc_job(c.byref(a),c.byref(j))==0
            c.memmove(j.peer_public,t.sc_wire_pub(peer_be),64)
            assert not lib.dbt_smp_worker_mbedtls_sc(c.byref(wa),c.byref(j),c.byref(r))
            assert bytes(r.ltk)==bytes(16) and bytes(r.local_check)==bytes(16)
        finally:
            lib.dbt_smp_worker_mbedtls_close(c.byref(wa)); lib.dbt_smp_worker_mbedtls_close(c.byref(wb))
    reject_peer(bytes(64))
    reject_peer(t.hx('20b003d2f297be2c5e2c83a7e9f9a5b9eff49111acf4fddbcc0301480e359de6'
                     'dc809c49652aeb6d63329abf5a52155c766345c28fed3024741c8ed01589d28b'))
    print('SDK mbedTLS host worker passed real P-256/AES-CMAC paired SC exchanges and replay checks')
finally:
    subprocess.run(check, check=True)
