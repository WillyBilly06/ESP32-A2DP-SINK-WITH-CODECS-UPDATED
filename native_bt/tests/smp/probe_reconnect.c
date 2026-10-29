/* Probe attachment, real NVS serialization and simulated controller commands. */
#define PROBE_REAL_NVS 1
#define main unused_probe_adapter_main
#include "probe_adapter.c"
#undef main

void fake_nvs_reset(void);
void fake_nvs_cut_after_commit(unsigned);
void fake_nvs_power_cycle(void);
static void seed(const dbt_link_t *l,bool sc,bool authenticated,unsigned key_size) {
    dbt_smp_nvs_t provider; dbt_smp_bond_job_t job={0};
    dbt_smp_nvs_init(&provider,1); job.op=DBT_SMP_BOND_STORE; job.generation=99; job.serial=1;
    job.identity_type=l->peer_address_type; memcpy(job.identity,l->peer,6);
    job.record.identity_type=job.identity_type; memcpy(job.record.identity,job.identity,6);
    job.record.key_size=(uint8_t)key_size; job.record.secure_connections=sc;
    job.record.authenticated=authenticated;
    if (!sc) {
        job.record.have_local_ltk=true; job.record.local_ediv=0x1234; job.record.local_rand[0]=0x42;
        job.record.have_peer_ltk=true; job.record.ediv=0x9876; job.record.rand[0]=0x33;
    }
    for (unsigned i=0;i<key_size;i++) {
        job.record.ltk[i]=(uint8_t)(0x20+i);
        if (!sc) job.record.local_ltk[i]=(uint8_t)(0x40+i);
    }
    assert(dbt_smp_nvs_execute(&provider,&job,NULL)==DBT_OK);
    wipe(&job,sizeof(job));
}
static bool request(uint16_t ediv,uint8_t first_rand) {
    uint8_t payload[13]={5,1,0}; payload[3]=first_rand; payload[11]=(uint8_t)ediv; payload[12]=(uint8_t)(ediv>>8);
    dbt_hci_packet_t packet={0}; packet.type=DBT_H4_EVENT; packet.event=0x3e; packet.payload=payload; packet.size=13;
    return probe_smp_event(&packet);
}
static void encryption(uint8_t status,uint8_t enabled) {
    uint8_t payload[4]={status,1,0,enabled};
    dbt_hci_packet_t packet={0}; packet.type=DBT_H4_EVENT; packet.event=8; packet.payload=payload; packet.size=4;
    assert(!probe_smp_event(&packet));
}
static void command_complete(uint16_t opcode) {
    assert(r.host.command_count==1);
    dbt_command_t *c=&r.host.commands[r.host.command_head]; assert(c->opcode==opcode);
    dbt_command_done_t done=c->done; void *ctx=c->context; uint8_t ret[3]={0,1,0};
    wipe(c->parameters,sizeof(c->parameters)); r.host.command_count=0; r.host.pending_opcode=0;
    done(ctx,opcode,DBT_COMMAND_SUCCESS,0,ret,sizeof(ret));
}
static void command_status(uint16_t opcode) {
    assert(r.host.command_count==1);
    dbt_command_t *c=&r.host.commands[r.host.command_head]; assert(c->opcode==opcode && c->response==DBT_COMMAND_STATUS);
    dbt_command_done_t done=c->done; void *ctx=c->context; uint8_t ret[1]={0};
    wipe(c->parameters,sizeof(c->parameters)); r.host.command_count=0; r.host.pending_opcode=0;
    done(ctx,opcode,DBT_COMMAND_SUCCESS,0,ret,sizeof(ret));
}
static void finish(uint32_t now) {
    dbt_link_t *l=&r.acl.links[0]; dbt_link_change_t change={DBT_LINK_REMOVED,1,l->generation};
    probe_smp_link(&change); l->used=false;
    probe_smp_poll(now);
    if (work_inflight) { run_worker(); probe_smp_poll(now+1); }
    if (work_inflight) { run_worker(); probe_smp_poll(now+2); }
    assert(!slot.active && !worker_live && !reconnect);
}
static void next_link(unsigned generation,uint8_t peer) {
    dbt_link_t *l=&r.acl.links[0]; l->used=true; l->encrypted=false; l->generation=generation; l->peer[0]=peer;
    r.now_ms=generation*10000u;
}
static uint8_t *fail_allocate(void *ctx,uint16_t bytes) { (void)ctx; (void)bytes; return NULL; }
static void negative_done(uint32_t now,unsigned generation) {
    assert(slot.reply_pending && slot.reply_opcode==0x201b);
    command_complete(0x201b); probe_smp_poll(now);
    if (work_inflight) { run_worker(); probe_smp_poll(now+1); }
    assert(!slot.active);
    uint16_t h; uint32_t gen; assert(probe_smp_take_failure(&h,&gen) && gen==generation);
}
static void maintenance_run(uint32_t now,dbt_result_t *status,dbt_smp_bond_op_t *op,
                            uint8_t *type,uint8_t id[6],bool *applied) {
    probe_smp_poll(now); assert(work_inflight); run_worker(); probe_smp_poll(now+1);
    assert(work_inflight); run_worker(); probe_smp_poll(now+2);
    assert(probe_smp_storage_result(status,op,type,id,applied)==DBT_OK);
    assert(!maintenance.active && !worker_live);
}
int main(void) {
    r.ready=true; r.host.state=DBT_HOST_IDENTIFIED; r.host.timeout_ms=2000; r.maximum_mtu=128;
    r.hooks.allocate=allocate; r.hooks.release=release; r.channels.hooks.fixed_data=fixed;
    dbt_link_t *l=&r.acl.links[0]; l->used=true; l->kind=DBT_LINK_LE; l->handle=1;
    l->generation=1; l->role=1; l->peer_address_type=0; l->peer[0]=7;
    probe_smp_config_t cfg={0}; cfg.local_address[0]=1; r.host.controller.address_le[0]=1;
    cfg.pairing.features.io=3; cfg.pairing.features.min_key_size=16; cfg.pairing.features.max_key_size=16;
    cfg.pairing.features.support_sc=true; cfg.pairing.features.require_sc=true; cfg.pairing.features.bonding=true;
    assert(probe_smp_init(&r,(TaskHandle_t)1,&cfg)==DBT_OK);
    fake_nvs_reset(); seed(l,true,false,16);
    assert(request(0,0)); assert(!worker_live); probe_smp_poll(0);
    assert(work_inflight && !slot.reply_pending); run_worker(); probe_smp_poll(1);
    assert(slot.reply_pending && slot.reply_opcode==0x201a);
    dbt_command_t *cmd=&r.host.commands[r.host.command_head];
    for (unsigned i=0;i<16;i++) assert(cmd->parameters[2+i]==0x20+i);
    dbt_smp_session_security_t sec;
    assert(probe_smp_security(1,1,&sec)==DBT_OK && !sec.encrypted && !sec.peer_key_verified);
    command_complete(0x201a);
    assert(probe_smp_security(1,1,&sec)==DBT_OK && !sec.encrypted);
    l->encrypted=true; encryption(0,1);
    assert(probe_smp_security(1,1,&sec)==DBT_OK && sec.encrypted && !sec.authenticated &&
           sec.bond_committed && sec.peer_key_verified && sec.key_size==16 && !sec.final_ll_acked &&
           !sec.phase3_complete && !sec.negotiated_local_keys && !sec.negotiated_peer_keys);
    probe_smp_poll(2); assert(work_inflight); run_worker(); probe_smp_poll(3);
    assert(slot.active && slot.verified && slot.retired && !worker_live);
    finish(4);

    /* A result from the previous generation cannot produce an LTK reply. */
    next_link(2,7); assert(request(0,0)); probe_smp_poll(20000);
    dbt_link_change_t change={DBT_LINK_REMOVED,1,2}; probe_smp_link(&change); l->used=false;
    run_worker(); probe_smp_poll(20001); assert(!r.host.command_count && !slot.verified);
    run_worker(); probe_smp_poll(20002); assert(!slot.active);

    /* A real stored weak key is rejected against this connection's policy. */
    next_link(3,8); seed(l,true,false,7); assert(request(0,0)); probe_smp_poll(30000);
    run_worker(); probe_smp_poll(30001); assert(slot.reply_pending && slot.reply_opcode==0x201b);
    command_complete(0x201b); probe_smp_poll(30002); run_worker(); probe_smp_poll(30003); assert(!slot.active);
    uint16_t h; uint32_t gen; assert(probe_smp_take_failure(&h,&gen) && gen==3);

    /* Identifier mismatch for a stored legacy bond never releases its key. */
    next_link(4,9); configuration.pairing.features.require_sc=false; seed(l,false,true,16);
    assert(request(0x1234,0x43)); probe_smp_poll(40000); run_worker(); probe_smp_poll(40001);
    assert(slot.reply_pending && slot.reply_opcode==0x201b);
    command_complete(0x201b); probe_smp_poll(40002); run_worker(); probe_smp_poll(40003);
    assert(!slot.active); assert(probe_smp_take_failure(&h,&gen) && gen==4);

    /* An encryption event before command completion cannot prove reconnect. */
    next_link(5,10); configuration.pairing.features.require_sc=true; seed(l,true,true,16);
    assert(request(0,0)); probe_smp_poll(50000); run_worker(); probe_smp_poll(50001);
    assert(slot.reply_pending && !slot.reply_accepted); encryption(0,1);
    assert(slot.failed && !slot.verified && !slot.reply_pending && !r.host.command_count);
    probe_smp_poll(50002); run_worker(); probe_smp_poll(50003); assert(!slot.active);
    assert(probe_smp_take_failure(&h,&gen) && gen==5);

    /* A storage result arriving after timeout is discarded; send negative. */
    next_link(6,11); seed(l,true,false,16); assert(request(0,0)); probe_smp_poll(60000);
    probe_smp_poll(62000); assert(slot.negative_needed && slot.reply_pending && slot.reply_opcode==0x201b);
    run_worker(); probe_smp_poll(62001); assert(!slot.key_ready);
    command_complete(0x201b); probe_smp_poll(62002); run_worker(); probe_smp_poll(62003);
    assert(!slot.active); assert(probe_smp_take_failure(&h,&gen) && gen==6);

    /* Reconnect cannot satisfy an authenticated-only policy with Just Works. */
    next_link(7,12); configuration.pairing.features.require_mitm=true; seed(l,true,false,16);
    assert(request(0,0)); probe_smp_poll(70000); run_worker(); probe_smp_poll(70001);
    negative_done(70002,7); configuration.pairing.features.require_mitm=false;

    /* A Legacy record cannot satisfy an SC-only policy. */
    next_link(8,13); seed(l,false,true,16); assert(request(0x1234,0x42));
    probe_smp_poll(80000); run_worker(); probe_smp_poll(80001); negative_done(80002,8);

    /* With Legacy permitted, the responder uses its own stored LTK and IDs. */
    next_link(9,14); configuration.pairing.features.require_sc=false; seed(l,false,true,16);
    assert(request(0x1234,0x42)); probe_smp_poll(90000); run_worker(); probe_smp_poll(90001);
    assert(slot.reply_pending && slot.reply_opcode==0x201a);
    cmd=&r.host.commands[r.host.command_head];
    for (unsigned i=0;i<16;i++) assert(cmd->parameters[2+i]==0x40+i);
    command_complete(0x201a); l->encrypted=true; encryption(0,1);
    assert(probe_smp_security(1,9,&sec)==DBT_OK && sec.encrypted && sec.authenticated && sec.peer_key_verified);
    encryption(0,0); assert(slot.failed);
    assert(probe_smp_security(1,9,&sec)==DBT_OK && !sec.encrypted && !sec.peer_key_verified);
    finish(90002);
    assert(probe_smp_take_failure(&h,&gen) && gen==9);

    /* No matching record or arena allocation produces a negative reply. */
    next_link(10,15); fake_nvs_reset(); assert(request(0,0));
    probe_smp_poll(100000); run_worker(); probe_smp_poll(100001); negative_done(100002,10);
    next_link(11,16); r.hooks.allocate=fail_allocate; assert(request(0,0));
    probe_smp_poll(110000); assert(slot.negative_needed && !work_inflight);
    probe_smp_poll(110001); negative_done(110002,11); r.hooks.allocate=allocate;

    /* Resolvable private addresses are unclaimed, leaving GAP to decline. */
    next_link(12,17); l->peer_address_type=1; l->peer[5]=0x40;
    assert(!request(0,0) && !slot.active); l->peer_address_type=0;

    /* Already submitted replies remain owned until completion after disconnect. */
    next_link(13,18); seed(l,true,false,16); assert(request(0,0));
    probe_smp_poll(130000); run_worker(); probe_smp_poll(130001);
    assert(slot.reply_pending && r.host.command_count==1);
    r.host.pending_opcode=0x201a;
    change.kind=DBT_LINK_REMOVED; change.handle=1; change.generation=13;
    probe_smp_link(&change); l->used=false;
    assert(r.host.state==DBT_HOST_IDENTIFIED && r.host.command_count==1 && slot.reply_pending);
    probe_smp_poll(130002); assert(slot.active);
    command_complete(0x201a); probe_smp_poll(130003); run_worker(); probe_smp_poll(130004);
    assert(!slot.active && quarantined(1));
    /* Test-only simulated controller reboot before the next independent case. */
    quarantine_count=0; quarantine_all=false; memset(quarantined_handles,0,sizeof(quarantined_handles));

    /* A queued unsent LTK reply must not escape after handle reuse. */
    next_link(14,19); seed(l,true,false,16); assert(request(0,0));
    probe_smp_poll(140000); run_worker(); probe_smp_poll(140001);
    assert(slot.reply_pending && r.host.command_count==1 && !r.host.pending_opcode);
    change.kind=DBT_LINK_REMOVED; change.handle=1; change.generation=14;
    probe_smp_link(&change); l->used=false;
    assert(r.host.state==DBT_HOST_IDENTIFIED && r.host.command_count==0 && !slot.reply_pending);
    for (unsigned i=0;i<sizeof(r.host.commands[0].parameters);i++) assert(r.host.commands[0].parameters[i]==0);
    probe_smp_poll(140002); run_worker(); probe_smp_poll(140003); assert(!slot.active);

    /* Central SC reconnect is explicit and uses the stored SC LTK with zero IDs. */
    next_link(15,20); l->role=0; configuration.pairing.features.require_sc=true; seed(l,true,true,16);
    assert(probe_smp_reconnect_start(l)==DBT_OK); probe_smp_poll(150000); run_worker(); probe_smp_poll(150001);
    assert(slot.reply_pending && slot.reply_opcode==0x2019);
    cmd=&r.host.commands[r.host.command_head]; assert(cmd->size==28);
    for (unsigned i=2;i<12;i++) assert(cmd->parameters[i]==0);
    for (unsigned i=0;i<16;i++) assert(cmd->parameters[12+i]==0x20+i);
    assert(probe_smp_security(1,15,&sec)==DBT_OK && !sec.encrypted);
    command_status(0x2019); assert(probe_smp_security(1,15,&sec)==DBT_OK && !sec.encrypted);
    l->encrypted=true; encryption(0,1);
    assert(probe_smp_security(1,15,&sec)==DBT_OK && sec.encrypted && sec.authenticated && sec.peer_key_verified);
    finish(150002);

    /* Legacy central uses the peer's LTK and peer EDIV/Rand, not local keys. */
    next_link(16,21); l->role=0; configuration.pairing.features.require_sc=false; seed(l,false,true,16);
    assert(probe_smp_reconnect_start(l)==DBT_OK); probe_smp_poll(160000); run_worker(); probe_smp_poll(160001);
    cmd=&r.host.commands[r.host.command_head]; assert(cmd->opcode==0x2019 && cmd->parameters[2]==0x33 &&
                                                       cmd->parameters[10]==0x76 && cmd->parameters[11]==0x98);
    for (unsigned i=0;i<16;i++) assert(cmd->parameters[12+i]==0x20+i);
    command_status(0x2019); l->encrypted=true; encryption(0,1); assert(slot.verified); finish(160002);

    /* Missing central record fails without sending an HCI command. */
    next_link(17,22); l->role=0; fake_nvs_reset(); assert(probe_smp_reconnect_start(l)==DBT_OK);
    probe_smp_poll(170000); run_worker(); probe_smp_poll(170001); assert(slot.failed && !r.host.command_count);
    probe_smp_poll(170002); run_worker(); probe_smp_poll(170003); assert(!slot.active);
    assert(probe_smp_take_failure(&h,&gen) && gen==17);

    /* Stale worker results cannot start central encryption after disconnect. */
    next_link(18,23); l->role=0; seed(l,true,false,16); assert(probe_smp_reconnect_start(l)==DBT_OK);
    probe_smp_poll(180000); change.generation=18; probe_smp_link(&change); l->used=false;
    run_worker(); probe_smp_poll(180001); assert(!r.host.command_count && !slot.verified);
    run_worker(); probe_smp_poll(180002); assert(!slot.active);

    /* Unsent central encryption commands are cancelled and wiped on disconnect. */
    next_link(19,24); l->role=0; seed(l,true,false,16); assert(probe_smp_reconnect_start(l)==DBT_OK);
    probe_smp_poll(190000); run_worker(); probe_smp_poll(190001);
    assert(slot.reply_pending && !r.host.pending_opcode); change.generation=19; probe_smp_link(&change); l->used=false;
    assert(r.host.state==DBT_HOST_IDENTIFIED && !r.host.command_count && !slot.reply_pending);
    probe_smp_poll(190002); run_worker(); probe_smp_poll(190003); assert(!slot.active);

    /* Submitted central commands are retained until their late status. */
    next_link(20,25); l->role=0; seed(l,true,false,16); assert(probe_smp_reconnect_start(l)==DBT_OK);
    probe_smp_poll(200000); run_worker(); probe_smp_poll(200001); r.host.pending_opcode=0x2019;
    change.generation=20; probe_smp_link(&change); l->used=false;
    assert(slot.reply_pending && r.host.command_count==1); probe_smp_poll(200002); assert(slot.active);
    command_status(0x2019); probe_smp_poll(200003); run_worker(); probe_smp_poll(200004); assert(!slot.active);
    assert(quarantined(1));
    next_link(21,26); l->role=0; assert(probe_smp_reconnect_start(l)==DBT_INVALID);

    /* Explicit worker-only recovery verifies and clears the persistent intent. */
    fake_nvs_reset(); seed(l,true,false,16); dbt_smp_nvs_t provider; dbt_smp_nvs_init(&provider,1);
    dbt_smp_bond_job_t mutation={0}; mutation.op=DBT_SMP_BOND_STORE; mutation.generation=50; mutation.serial=1;
    mutation.identity_type=0; memcpy(mutation.identity,l->peer,6); mutation.record.identity_type=0;
    memcpy(mutation.record.identity,l->peer,6); mutation.record.key_size=16; mutation.record.secure_connections=true;
    memset(mutation.record.ltk,0x27,16);
    fake_nvs_cut_after_commit(2); assert(dbt_smp_nvs_execute(&provider,&mutation,NULL)==DBT_INVALID);
    fake_nvs_power_cycle(); storage_poisoned=true;
    assert(probe_smp_storage_inspect_start()==DBT_OK);
    dbt_result_t status; dbt_smp_bond_op_t op; uint8_t type,id[6]; bool applied;
    maintenance_run(210000,&status,&op,&type,id,&applied);
    assert(status==DBT_OK && op==DBT_SMP_BOND_STORE && type==0 && !memcmp(id,l->peer,6) && storage_poisoned);
    uint8_t wrong[6]; memcpy(wrong,id,6); wrong[0]^=1;
    assert(probe_smp_storage_reconcile_start(op,type,wrong)==DBT_OK);
    maintenance_run(210010,&status,&op,&type,wrong,&applied);
    assert(status==DBT_INVALID && storage_poisoned);
    assert(probe_smp_storage_reconcile_start(DBT_SMP_BOND_STORE,0,l->peer)==DBT_OK);
    maintenance_run(210020,&status,&op,&type,id,&applied);
    assert(status==DBT_OK && applied && !storage_poisoned);
    assert(probe_smp_storage_inspect_start()==DBT_OK);
    maintenance_run(210030,&status,&op,&type,id,&applied); assert(status==DBT_EMPTY);
    /* A cut after intent but before mutation reconciles the verified before
     * state and reports applied=false. */
    fake_nvs_reset(); seed(l,true,false,16); dbt_smp_nvs_init(&provider,1);
    fake_nvs_cut_after_commit(1); assert(dbt_smp_nvs_execute(&provider,&mutation,NULL)==DBT_INVALID);
    fake_nvs_power_cycle(); storage_poisoned=true;
    assert(probe_smp_storage_reconcile_start(DBT_SMP_BOND_STORE,0,l->peer)==DBT_OK);
    maintenance_run(210040,&status,&op,&type,id,&applied);
    assert(status==DBT_OK && !applied && !storage_poisoned);
    wipe(&mutation,sizeof(mutation));

    /* Privacy: a known candidate IRK resolves on the worker and requires a
     * correlated encryption plus durable promotion before final success. */
    quarantine_count=0; quarantine_all=false; memset(quarantined_handles,0,sizeof(quarantined_handles));
    configuration.peer_identity_only=true; configuration.pairing.features.require_sc=true;
    const uint8_t irk[16]={0x9b,0x7d,0x39,0x0a,0xa6,0x10,0x10,0x34,0x05,0xad,0xc8,0x57,0xa3,0x34,0x02,0xec};
    const uint8_t identity[6]={1,2,3,4,5,6};
    const uint8_t address1[6]={0xaa,0xfb,0x0d,0x94,0x81,0x70};
    const uint8_t address2[6]={0xf7,0x93,0xbd,0x45,0x23,0x71};
    fake_nvs_reset(); dbt_smp_nvs_init(&provider,1); memset(&mutation,0,sizeof(mutation));
    mutation.op=DBT_SMP_BOND_STORE; mutation.generation=99; mutation.serial=1;
    memcpy(mutation.identity,identity,6); mutation.record.identity_type=0;
    memcpy(mutation.record.identity,identity,6); mutation.record.key_size=16; mutation.record.secure_connections=true;
    memset(mutation.record.ltk,0x26,16); mutation.record.have_irk=true; memcpy(mutation.record.irk,irk,16);
    mutation.record.candidate_present=true; mutation.record.candidate_type=0;
    memcpy(mutation.record.candidate_identity,identity,6); memcpy(mutation.record.candidate_irk,irk,16);
    assert(dbt_smp_nvs_execute(&provider,&mutation,NULL)==DBT_OK);
    next_link(22,0); l->role=1; l->peer_address_type=1; memcpy(l->peer,address1,6);
    assert(request(0,0)); probe_smp_poll(220000); run_worker(); probe_smp_poll(220001);
    assert(slot.reply_pending && slot.reply_opcode==0x201a); command_complete(0x201a);
    l->encrypted=true; encryption(0,1); assert(slot.encrypted_verified && !slot.verified);
    assert(probe_smp_security(1,22,&sec)==DBT_OK && sec.encrypted && sec.peer_key_verified &&
           !sec.identity_promotion_committed && !sec.authenticated);
    probe_smp_poll(220002); assert(work_inflight); run_worker(); probe_smp_poll(220003);
    assert(slot.verified && probe_smp_security(1,22,&sec)==DBT_OK && sec.identity_promotion_committed);
    if (work_inflight) { run_worker(); probe_smp_poll(220004); }
    finish(220005);
    next_link(23,0); l->role=1; l->peer_address_type=1; memcpy(l->peer,address2,6);
    assert(request(0,0)); probe_smp_poll(230000); run_worker(); probe_smp_poll(230001);
    assert(slot.reply_pending && slot.reply_opcode==0x201a); command_complete(0x201a);
    l->encrypted=true; encryption(0,1);
    assert(slot.verified && probe_smp_security(1,23,&sec)==DBT_OK && sec.identity_promotion_committed);
    finish(230002);
    next_link(24,0); l->role=1; l->peer_address_type=1; memcpy(l->peer,address2,6); l->peer[0]^=1;
    assert(request(0,0)); probe_smp_poll(240000); run_worker(); probe_smp_poll(240001);
    negative_done(240002,24);
    next_link(25,0); l->role=1; l->peer_address_type=1; memcpy(l->peer,address1,6);
    assert(request(0,0)); probe_smp_poll(250000); change.generation=25; probe_smp_link(&change); l->used=false;
    run_worker(); probe_smp_poll(250001); assert(!r.host.command_count && !slot.verified);
    run_worker(); probe_smp_poll(250002); assert(!slot.active);
    /* A late durable promotion may have storage effects, but it cannot
     * qualify the timed-out live generation. */
    fake_nvs_reset(); dbt_smp_nvs_init(&provider,1);
    assert(dbt_smp_nvs_execute(&provider,&mutation,NULL)==DBT_OK);
    next_link(26,0); l->role=1; l->peer_address_type=1; memcpy(l->peer,address1,6);
    assert(request(0,0)); probe_smp_poll(260000); run_worker(); probe_smp_poll(260001);
    command_complete(0x201a); l->encrypted=true; encryption(0,1);
    probe_smp_poll(260002); assert(work_inflight); run_worker(); probe_smp_poll(262001);
    assert(slot.failed && !slot.verified);
    assert(probe_smp_security(1,26,&sec)==DBT_OK && !sec.encrypted && !sec.identity_promotion_committed);
    probe_smp_poll(262002); run_worker(); probe_smp_poll(262003); assert(!slot.active);
    wipe(&mutation,sizeof(mutation));
    return 0;
}
