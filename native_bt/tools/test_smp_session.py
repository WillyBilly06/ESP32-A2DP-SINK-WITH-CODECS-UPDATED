"""Pair two real portable runtimes through an in-process HCI controller.

The controller is a software model, not RF. The crypto worker is the separately
built SDK mbedTLS implementation with the host CSPRNG. Bond commits are mocked.
"""
import ctypes as c
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/smp'
check=[sys.executable,str(ROOT/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    import test_smp_worker as w
    t=w.t; windows=os.name=='nt'
    cc=os.environ.get('CC') or (r'C:\Program Files\LLVM\bin\clang.exe' if windows else shutil.which('cc'))
    if not cc: raise RuntimeError('A C compiler is required')
    sources=['queue','hci','host','acl','l2cap','credit','ertm','ertm_engine','channels','rx_flow','runtime',
             'smp','smp_crypto','smp_bond','smp_guard','smp_dist','smp_features','smp_sc','smp_bearer','smp_session']
    exports=['open','step','generation','crypto','crypto_done','legacy_crypto','legacy_crypto_done','numeric_value','numeric_confirm','store','store_done','security','phase','errors',
             'acl_count','bearer_size','key_command_accepted','key_command_pending','key_command_submitted','host_ok','uncertain','disconnect','identity_ready','identity_burst','identity_overflow','burst_on_encryption','burst_injected','rx_oom','poll_only','runtime_only','rx_zero','invalid_ltk','negative_replies','jump','inject_encryption','inject_ltk',
             'final_token','final_ack','try_close','close']
    dll=OUT/('smp_session_runtime.dll' if windows else 'smp_session_runtime.so')
    san=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('DBT_SMP_SANITIZE')=='1' and not windows else []
    flags=['-Wl,/export:smp_fixture_'+x for x in exports] if windows else ['-fPIC']
    build_env=os.environ.copy()
    if san: build_env.pop('LD_PRELOAD',None)
    subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror','-shared',*san,'-I',str(ROOT/'include'),
        *[str(ROOT/f'src/dbt_{name}.c') for name in sources],str(ROOT/'tests/smp/session_runtime.c'),
        *flags,'-o',str(dll)],check=True,env=build_env)
    lib=c.CDLL(str(dll))
    class Security(c.Structure):
        _fields_=[('encrypted',c.c_bool),('authenticated',c.c_bool),('bond_committed',c.c_bool),
            ('peer_key_verified',c.c_bool),('peer_final_received',c.c_bool),('final_ll_acked',c.c_bool),
            ('key_size',t.U8),('negotiated_local_keys',t.U8),('negotiated_peer_keys',t.U8),
            ('phase3_complete',c.c_bool),('identity_promotion_committed',c.c_bool),('admitted_local_keys',t.U8),('received_peer_keys',t.U8)]
    lib.smp_fixture_open.argtypes=[c.c_int,c.c_int,c.c_int,c.c_int]
    lib.smp_fixture_generation.argtypes=[c.c_uint]; lib.smp_fixture_generation.restype=c.c_uint32
    lib.smp_fixture_crypto.argtypes=[c.c_uint,c.POINTER(t.SCJob)]
    lib.smp_fixture_crypto_done.argtypes=[c.c_uint,c.c_uint32,c.c_uint32,c.c_bool,c.POINTER(t.SCResult)]
    lib.smp_fixture_legacy_crypto.argtypes=[c.c_uint,c.POINTER(t.Job)]
    lib.smp_fixture_legacy_crypto_done.argtypes=[c.c_uint,c.c_uint32,c.c_uint32,c.c_bool,c.POINTER(t.Result)]
    lib.smp_fixture_numeric_value.argtypes=[c.c_uint,c.POINTER(c.c_uint32)]
    lib.smp_fixture_numeric_confirm.argtypes=[c.c_uint,c.c_bool]
    lib.smp_fixture_store.argtypes=[c.c_uint,c.POINTER(t.BondJob)]
    lib.smp_fixture_store_done.argtypes=[c.c_uint,c.c_uint32,c.c_uint32,c.c_bool]
    lib.smp_fixture_security.argtypes=[c.c_uint,c.POINTER(Security)]
    lib.smp_fixture_inject_encryption.argtypes=[c.c_uint]
    lib.smp_fixture_inject_ltk.argtypes=[c.c_uint]
    lib.smp_fixture_disconnect.argtypes=[c.c_uint]
    lib.smp_fixture_identity_burst.argtypes=[c.c_uint]
    lib.smp_fixture_identity_overflow.argtypes=[c.c_uint]
    lib.smp_fixture_poll_only.argtypes=[c.c_uint]
    lib.smp_fixture_runtime_only.argtypes=[c.c_uint]
    lib.smp_fixture_final_token.argtypes=[c.c_uint,c.POINTER(c.c_uint32)]
    lib.smp_fixture_final_ack.argtypes=[c.c_uint,c.c_uint32,c.c_uint32]
    lib.smp_fixture_try_close.argtypes=[c.c_uint]
    lib.smp_fixture_jump.argtypes=[c.c_uint32]
    for name in ('phase','errors','acl_count','bearer_size','key_command_accepted','key_command_pending','key_command_submitted','host_ok','uncertain','identity_ready','rx_zero'):
        getattr(lib,'smp_fixture_'+name).argtypes=[c.c_uint]
        getattr(lib,'smp_fixture_'+name).restype=c.c_uint
    def run_pair(bonding,mode=0):
        assert lib.smp_fixture_open(int(bonding),0,0,mode)==0
        workers=[w.Worker(),w.Worker()]
        try:
            for i in range(2):
                assert w.lib.dbt_smp_worker_mbedtls_open(c.byref(workers[i]),lib.smp_fixture_generation(i),w.random_bytes,None)
            stores={}; saw_admission_without_encryption=False; done=False; committed=not bonding; numeric_confirmed=False
            for _ in range(1200):
                lib.smp_fixture_step()
                for i in range(2):
                    assert lib.smp_fixture_errors(i)==0 and lib.smp_fixture_phase(i)!=4
                    job=t.Job() if mode==1 else t.SCJob()
                    rc=(lib.smp_fixture_legacy_crypto(i,c.byref(job)) if mode==1 else lib.smp_fixture_crypto(i,c.byref(job)))
                    if rc==0:
                        result=t.Result() if mode==1 else t.SCResult()
                        execute=w.lib.dbt_smp_worker_mbedtls_legacy if mode==1 else w.lib.dbt_smp_worker_mbedtls_sc
                        complete=lib.smp_fixture_legacy_crypto_done if mode==1 else lib.smp_fixture_crypto_done
                        assert execute(c.byref(workers[i]),c.byref(job),c.byref(result))
                        assert complete(i,job.generation+1,job.serial,True,c.byref(result))==2
                        assert complete(i,job.generation,job.serial,True,c.byref(result))==0
                    else: assert rc in (1,2,5)
                    sec=Security(); sec_rc=lib.smp_fixture_security(i,c.byref(sec))
                    if sec_rc==0 and lib.smp_fixture_key_command_accepted(i) and not sec.encrypted:
                        saw_admission_without_encryption=True
                    if bonding and not committed and i not in stores:
                        store=t.BondJob(); src=lib.smp_fixture_store(i,c.byref(store))
                        if src==0: stores[i]=store
                        else: assert src in (1,2,5)
                if mode==2 and not numeric_confirmed:
                    av,bv=c.c_uint32(),c.c_uint32()
                    if lib.smp_fixture_numeric_value(0,c.byref(av))==0 and lib.smp_fixture_numeric_value(1,c.byref(bv))==0:
                        assert av.value==bv.value and av.value<1000000
                        assert lib.smp_fixture_numeric_confirm(0,True)==0
                        assert lib.smp_fixture_numeric_confirm(1,True)==0
                        numeric_confirmed=True
                if bonding and not committed and len(stores)==2:
                    for i in range(2):
                        sec=Security(); assert lib.smp_fixture_security(i,c.byref(sec))==0
                        assert sec.encrypted and not sec.bond_committed and sec.authenticated==(mode!=0)
                        assert not sec.peer_key_verified and sec.phase3_complete
                        assert sec.negotiated_local_keys==sec.negotiated_peer_keys==0
                        assert sec.admitted_local_keys==sec.received_peer_keys==0
                    for i in range(2):
                        st=stores[i]; assert lib.smp_fixture_store_done(i,st.generation+1,st.serial,True)==2
                        assert lib.smp_fixture_store_done(i,st.generation,st.serial,True)==0
                    committed=True
                statuses=[Security(),Security()]
                if all(lib.smp_fixture_security(i,c.byref(statuses[i]))==0 for i in range(2)) and all(s.encrypted for s in statuses):
                    if not committed: continue
                    assert all(s.authenticated==(mode!=0) and not s.peer_key_verified and s.key_size==16 for s in statuses)
                    assert all(s.phase3_complete and s.negotiated_local_keys==s.negotiated_peer_keys==0 for s in statuses)
                    assert all(s.admitted_local_keys==s.received_peer_keys==0 for s in statuses)
                    assert all(s.bond_committed==bonding for s in statuses)
                    assert all(lib.smp_fixture_bearer_size(i)==c.sizeof(t.Bearer if mode==1 else t.SCBearer) for i in range(2))
                    assert all(lib.smp_fixture_acl_count(i)>(2 if mode==1 else 4) for i in range(2))
                    assert saw_admission_without_encryption
                    done=True; break
            assert done, 'paired-runtime handshake did not complete'
        finally:
            for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
            assert lib.smp_fixture_close()==0
    run_pair(False); run_pair(True); run_pair(False,1); run_pair(False,2); run_pair(False,3)
    # The feature-to-engine allocation is lazy and allocation failure fails closed.
    assert lib.smp_fixture_open(0,1,0,0)==0
    for _ in range(100):
        lib.smp_fixture_step()
        if any(lib.smp_fixture_phase(i)==4 for i in range(2)): break
    assert any(lib.smp_fixture_phase(i)==4 for i in range(2))
    assert lib.smp_fixture_close()==0
    # An unsolicited Encryption Change cannot establish session security.
    assert lib.smp_fixture_open(0,0,0,0)==0
    assert lib.smp_fixture_inject_encryption(0)==0
    for _ in range(4): lib.smp_fixture_step()
    assert lib.smp_fixture_phase(0)==4
    assert lib.smp_fixture_close()==0
    # An LTK request received before feature exchange is explicitly declined.
    assert lib.smp_fixture_open(0,0,0,0)==0
    assert lib.smp_fixture_inject_ltk(1)==0
    for _ in range(20):
        lib.smp_fixture_step()
        if lib.smp_fixture_negative_replies(): break
    assert lib.smp_fixture_negative_replies()==1
    assert lib.smp_fixture_close()==0
    # Closing while the runtime still borrows a PDU must wait for its callback.
    assert lib.smp_fixture_open(0,0,0,0)==0
    lib.smp_fixture_step()
    assert lib.smp_fixture_try_close(0)==5
    assert lib.smp_fixture_close()==0
    def make_workers():
        workers=[w.Worker(),w.Worker()]
        for i in range(2):
            assert w.lib.dbt_smp_worker_mbedtls_open(c.byref(workers[i]),lib.smp_fixture_generation(i),w.random_bytes,None)
        return workers
    def service_crypto(workers):
        for i in range(2):
            job=t.SCJob(); rc=lib.smp_fixture_crypto(i,c.byref(job))
            if rc==0:
                result=t.SCResult(); assert w.lib.dbt_smp_worker_mbedtls_sc(c.byref(workers[i]),c.byref(job),c.byref(result))
                assert lib.smp_fixture_crypto_done(i,job.generation,job.serial,True,c.byref(result))==0
            else: assert rc in (1,2,5)
    # Two phase-3 Identity PDUs arrive in one runtime poll (budget eight),
    # before any session/parent poll, and are consumed in wire order.
    assert lib.smp_fixture_open(1,0,2,4)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_identity_ready(1): break
        assert lib.smp_fixture_identity_ready(1)
        assert lib.smp_fixture_identity_burst(1)==2
        assert lib.smp_fixture_host_ok(1) and lib.smp_fixture_errors(1)==0
        lib.smp_fixture_poll_only(1)
        store=t.BondJob(); assert lib.smp_fixture_store(1,c.byref(store))==0
        assert bytes(store.record.identity)==t.hx('010203040506')
        assert bytes(store.record.candidate_irk)==bytes(t.le('ec0234a357c8ad05341010a60a397d9b'))
        status=Security(); assert lib.smp_fixture_security(1,c.byref(status))==0
        assert status.peer_final_received and status.phase3_complete and not status.bond_committed
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # Disconnect wipes both queued identity PDUs before any session dispatch.
    assert lib.smp_fixture_open(1,0,2,4)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_identity_ready(1): break
        assert lib.smp_fixture_identity_ready(1) and lib.smp_fixture_identity_burst(1)==2
        assert lib.smp_fixture_disconnect(1)==0; lib.smp_fixture_runtime_only(1)
        assert lib.smp_fixture_phase(1)==4 and lib.smp_fixture_rx_zero(1)
        assert lib.smp_fixture_store(1,c.byref(t.BondJob()))!=0
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # Allocation failure never leaves a partially opened session.
    assert lib.smp_fixture_rx_oom()==0
    # The controller can report encryption and deliver both identity PDUs in
    # the same runtime poll; encryption must be applied before dispatch.
    assert lib.smp_fixture_open(1,0,2,4)==0
    lib.smp_fixture_burst_on_encryption(); workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_burst_injected(): break
        assert lib.smp_fixture_burst_injected() and lib.smp_fixture_errors(1)==0
        store=t.BondJob(); assert lib.smp_fixture_store(1,c.byref(store))==0
        assert bytes(store.record.candidate_irk)==bytes(t.le('ec0234a357c8ad05341010a60a397d9b'))
        status=Security(); assert lib.smp_fixture_security(1,c.byref(status))==0
        assert status.encrypted and status.peer_final_received and status.phase3_complete
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # Nine undrained PDUs exceed the finite queue and fail closed; no silent loss.
    assert lib.smp_fixture_open(1,0,2,4)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_identity_ready(1): break
        assert lib.smp_fixture_identity_ready(1)
        assert lib.smp_fixture_identity_overflow(1)==0
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # A disconnect cancels an unsent key command without aborting the host.
    assert lib.smp_fixture_open(0,0,0,0)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_key_command_pending(0): break
        assert lib.smp_fixture_key_command_pending(0) and not lib.smp_fixture_key_command_submitted(0)
        assert lib.smp_fixture_disconnect(0)==0; lib.smp_fixture_step()
        assert lib.smp_fixture_host_ok(0) and not lib.smp_fixture_key_command_pending(0) and not lib.smp_fixture_uncertain(0)
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # First SC pairing from an RPA: the peer's encrypted identity keys bind the
    # responder's provisional record, without claiming a final TX ACK.
    assert lib.smp_fixture_open(1,0,2,4)==0
    workers=make_workers(); received=None; token=c.c_uint32()
    try:
        for _ in range(800):
            lib.smp_fixture_step(); service_crypto(workers)
            job=t.BondJob(); rc=lib.smp_fixture_store(1,c.byref(job))
            if rc==0: received=job
            elif rc not in (1,2,5): raise AssertionError(rc)
            if received and lib.smp_fixture_final_token(0,c.byref(token))==0: break
        assert received and token.value
        record=received.record
        assert record.identity_type==0 and bytes(record.identity)==t.hx('010203040506')
        assert record.have_irk and record.candidate_present and not record.candidate_verified
        assert bytes(record.irk)==bytes(record.candidate_irk)==bytes(t.le('ec0234a357c8ad05341010a60a397d9b'))
        responder=Security(); assert lib.smp_fixture_security(1,c.byref(responder))==0
        assert responder.encrypted and responder.peer_final_received and not responder.bond_committed
        assert not responder.authenticated and not responder.final_ll_acked
        # NVS is an in-memory model with the real adapter and journal.
        import test_smp_nvs as nv
        nv.lib.fake_nvs_reset(); provider=nv.Provider(); nv.lib.dbt_smp_nvs_init(c.byref(provider),1)
        assert nv.lib.dbt_smp_nvs_execute(c.byref(provider),c.byref(received),None)==0
        assert lib.smp_fixture_store_done(1,received.generation,received.serial,True)==0
        # Explicit synthetic ACK is only fixture evidence for the sender.
        assert lib.smp_fixture_final_ack(0,lib.smp_fixture_generation(0),token.value)==0
        for _ in range(100):
            lib.smp_fixture_step()
            j=t.BondJob(); rc=lib.smp_fixture_store(0,c.byref(j))
            if rc==0:
                assert lib.smp_fixture_store_done(0,j.generation,j.serial,True)==0; break
        else: raise AssertionError('sender did not reach mock storage')
        # Resolve another RPA generated from the same IRK; reconnect remains
        # provisional until the LTK encryption and journaled promotion receipt.
        prand=bytes.fromhex('712345'); key=bytes(record.irk)[::-1]
        enc=t.Cipher(t.algorithms.AES(key),t.modes.ECB()).encryptor()
        hashed=(enc.update(bytes(13)+prand)+enc.finalize())[-3:]
        rotated=t.arr(hashed[::-1]+prand[::-1]); resolved=t.BondRecord(); candidate=c.c_bool(False)
        assert nv.lib.dbt_smp_nvs_resolve_rpa(c.byref(provider),rotated,c.byref(resolved),c.byref(candidate))==0
        assert candidate.value and bytes(resolved.identity)==bytes(record.identity)
        reconnect=t.Reconnect(); identity=t.arr(bytes(resolved.identity)); promoted=t.arr(bytes(resolved.candidate_identity))
        assert t.lib.dbt_smp_reconnect_open_candidate_role(c.byref(reconnect),999,0,identity,0,promoted,16,False,True,True)==0
        job=t.BondJob(); assert t.lib.dbt_smp_reconnect_job(c.byref(reconnect),c.byref(job))==0
        loaded=t.BondRecord(); assert nv.lib.dbt_smp_nvs_execute(c.byref(provider),c.byref(job),c.byref(loaded))==0
        assert t.lib.dbt_smp_reconnect_complete(c.byref(reconnect),999,job.serial,True,c.byref(loaded))==0
        keyout=t.arr(bytes(16)); keysize=t.U8(); zeros=t.arr(bytes(8))
        assert t.lib.dbt_smp_reconnect_take(c.byref(reconnect),0,zeros,keyout,c.byref(keysize))==0
        assert bytes(keyout)==bytes(record.ltk) and not reconnect.peer_key_verified
        assert t.lib.dbt_smp_reconnect_encryption(c.byref(reconnect),1000,True)==2
        assert t.lib.dbt_smp_reconnect_encryption(c.byref(reconnect),999,True)==0
        assert reconnect.peer_key_verified and not reconnect.identity_promotion_committed
        assert t.lib.dbt_smp_reconnect_job(c.byref(reconnect),c.byref(job))==0
        assert nv.lib.dbt_smp_nvs_execute(c.byref(provider),c.byref(job),None)==0
        assert t.lib.dbt_smp_reconnect_complete(c.byref(reconnect),999,job.serial,True,None)==0
        assert reconnect.identity_promotion_committed
        assert nv.lib.dbt_smp_nvs_resolve_rpa(c.byref(provider),rotated,c.byref(resolved),c.byref(candidate))==0
        assert resolved.candidate_verified
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # An already-submitted command remains pending until its actual completion;
    # its late completion cannot qualify the disconnected generation.
    assert lib.smp_fixture_open(0,0,0,0)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_key_command_pending(0): break
        assert lib.smp_fixture_key_command_pending(0)
        lib.smp_fixture_step(); assert lib.smp_fixture_key_command_submitted(0)
        assert lib.smp_fixture_disconnect(0)==0; lib.smp_fixture_step()
        assert lib.smp_fixture_host_ok(0) and lib.smp_fixture_phase(0)==4 and lib.smp_fixture_uncertain(0)
        assert not lib.smp_fixture_key_command_pending(0)
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # Encryption Change while the key command is only queued cannot later be
    # converted into a successful correlated encryption result.
    assert lib.smp_fixture_open(0,0,0,0)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_key_command_pending(0): break
        assert lib.smp_fixture_key_command_pending(0) and not lib.smp_fixture_key_command_accepted(0)
        assert lib.smp_fixture_inject_encryption(0)==0
        lib.smp_fixture_step(); assert lib.smp_fixture_phase(0)==4
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # A nonzero LTK request is rejected with the negative HCI response.
    assert lib.smp_fixture_open(0,0,0,0)==0
    lib.smp_fixture_invalid_ltk(); workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            if lib.smp_fixture_negative_replies(): break
        assert lib.smp_fixture_negative_replies()==1
        for _ in range(4): lib.smp_fixture_step()
        assert lib.smp_fixture_phase(1)==4
        sec=Security(); assert lib.smp_fixture_security(1,c.byref(sec))!=0 or not sec.encrypted
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # A stale durable-storage receipt cannot complete after owner timeout.
    assert lib.smp_fixture_open(1,0,0,0)==0
    workers=make_workers(); pending_store=None
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            store=t.BondJob(); rc=lib.smp_fixture_store(0,c.byref(store))
            if rc==0: pending_store=store; break
            assert rc in (1,2,5)
        assert pending_store
        lib.smp_fixture_jump(6000); lib.smp_fixture_step()
        assert lib.smp_fixture_phase(0)==4
        assert lib.smp_fixture_store_done(0,pending_store.generation,pending_store.serial,True)==2
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # Storage must also time out if no worker ever claims the pending job.
    assert lib.smp_fixture_open(1,0,0,0)==0
    workers=make_workers()
    try:
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            sec=Security()
            if lib.smp_fixture_security(0,c.byref(sec))==0 and sec.encrypted: break
        assert sec.encrypted and not sec.bond_committed
        lib.smp_fixture_jump(6000); lib.smp_fixture_step()
        assert lib.smp_fixture_phase(0)==4
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    # With real runtime admission and synthetic HCI buffer credits, but no ACK
    # provider, the global final-key sender remains uncommitted and times out.
    assert lib.smp_fixture_open(1,0,1,0)==0
    workers=make_workers(); receiver_store=None
    try:
        token=c.c_uint32()
        for _ in range(600):
            lib.smp_fixture_step(); service_crypto(workers)
            store=t.BondJob(); rc=lib.smp_fixture_store(1,c.byref(store))
            if rc==0: receiver_store=store
            elif rc not in (1,2,5): raise AssertionError(rc)
            if receiver_store and lib.smp_fixture_final_token(0,c.byref(token))==0: break
        assert receiver_store and token.value
        a,b=Security(),Security()
        assert lib.smp_fixture_security(0,c.byref(a))==0 and lib.smp_fixture_security(1,c.byref(b))==0
        assert a.encrypted and b.encrypted and not a.bond_committed and not b.bond_committed
        assert not a.final_ll_acked and b.peer_final_received
        assert a.negotiated_local_keys==a.negotiated_peer_keys==4 and b.negotiated_local_keys==b.negotiated_peer_keys==4
        assert not a.phase3_complete and b.phase3_complete
        assert a.admitted_local_keys==a.received_peer_keys==0
        assert b.admitted_local_keys==b.received_peer_keys==4
        lib.smp_fixture_jump(31000); lib.smp_fixture_step()
        assert lib.smp_fixture_phase(0)==4
        assert lib.smp_fixture_store(0,c.byref(t.BondJob()))!=0
    finally:
        for worker in workers: w.lib.dbt_smp_worker_mbedtls_close(c.byref(worker))
        assert lib.smp_fixture_close()==0
    print('Paired runtimes: real worker, Identity ACL bursts, same-poll encryption, bounded queue and mock commits passed')
finally:
    subprocess.run(check,check=True)
