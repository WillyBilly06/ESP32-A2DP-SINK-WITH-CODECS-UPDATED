#include "dbt_smp_nvs.h"
#include "dbt_smp_crypto.h"
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include <string.h>
#define RECORD_BYTES 154u
#define INTENT_BYTES 110u
#define INTENT_KEY "smp_txn_v1"
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool valid(const dbt_smp_bond_record_t *r, uint8_t type, const uint8_t id[6]) {
  if (r->identity_type != type || type > 1 || memcmp(r->identity, id, 6) || r->key_size < 7 ||
      r->key_size > 16)
    return false;
  for (unsigned i = r->key_size; i < 16; i++)
    if (r->ltk[i])
      return false;
  for (unsigned i = r->key_size; i < 16; i++)
    if (r->local_ltk[i])
      return false;
  if (r->secure_connections && (r->have_local_ltk || r->have_peer_ltk))
    return false;
  if (r->secure_connections) {
    if (r->ediv)
      return false;
    for (unsigned i = 0; i < 8; i++)
      if (r->rand[i])
        return false;
  }
  if (r->candidate_verified && !r->candidate_present)
    return false;
  if (r->candidate_present &&
      (r->candidate_type > 1 || (r->candidate_type == 1 && (r->candidate_identity[5] & 0xc0) != 0xc0)))
    return false;
  if (r->have_irk && r->candidate_present && r->identity_type==r->candidate_type &&
      !memcmp(r->identity,r->candidate_identity,6) && memcmp(r->irk,r->candidate_irk,16))
    return false;
  return true;
}
static void make_key(char key[14], uint8_t type, const uint8_t id[6]) {
  static const char hex[] = "0123456789abcdef";
  key[0] = (char)('0' + type);
  for (unsigned i = 0; i < 6; i++) {
    key[1 + i * 2] = hex[id[i] >> 4];
    key[2 + i * 2] = hex[id[i] & 15];
  }
  key[13] = 0;
}
static void encode(const dbt_smp_bond_record_t *r, uint8_t out[RECORD_BYTES]) {
  unsigned p = 0;
  memcpy(out + p, "DBTB", 4);
  p += 4;
  out[p++] = 2;
  out[p++] = r->identity_type;
  memcpy(out + p, r->identity, 6);
  p += 6;
  memcpy(out + p, r->ltk, 16);
  p += 16;
  memcpy(out + p, r->rand, 8);
  p += 8;
  memcpy(out + p, r->irk, 16);
  p += 16;
  out[p++] = (uint8_t)r->ediv;
  out[p++] = (uint8_t)(r->ediv >> 8);
  out[p++] = r->key_size;
  out[p++] = (r->authenticated ? 1 : 0) | (r->secure_connections ? 2 : 0) | (r->have_irk ? 4 : 0) |
             (r->have_local_csrk ? 8 : 0) | (r->have_peer_csrk ? 16 : 0) | (r->have_local_irk ? 32 : 0) |
             (r->candidate_present ? 64 : 0) | (r->candidate_verified ? 128 : 0);
  memcpy(out + p, r->local_csrk, 16);
  p += 16;
  memcpy(out + p, r->peer_csrk, 16);
  p += 16;
  memcpy(out + p, r->local_irk, 16);
  p += 16;
  memcpy(out + p, r->candidate_irk, 16);
  p += 16;
  memcpy(out + p, r->candidate_identity, 6);
  p += 6;
  out[p++] = r->candidate_type;
  memcpy(out + p, r->local_ltk, 16);
  p += 16;
  memcpy(out + p, r->local_rand, 8);
  p += 8;
  out[p++] = (uint8_t)r->local_ediv;
  out[p++] = (uint8_t)(r->local_ediv >> 8);
  out[p] = (r->have_local_ltk ? 1 : 0) | (r->have_peer_ltk ? 2 : 0);
}
static bool decode(const uint8_t in[RECORD_BYTES], dbt_smp_bond_record_t *r) {
  if (memcmp(in, "DBTB", 4) || in[4] != 2)
    return false;
  unsigned p = 5;
  memset(r, 0, sizeof(*r));
  r->identity_type = in[p++];
  memcpy(r->identity, in + p, 6);
  p += 6;
  memcpy(r->ltk, in + p, 16);
  p += 16;
  memcpy(r->rand, in + p, 8);
  p += 8;
  memcpy(r->irk, in + p, 16);
  p += 16;
  r->ediv = (uint16_t)(in[p] | ((uint16_t)in[p + 1] << 8));
  p += 2;
  r->key_size = in[p++];
  uint8_t f = in[p++];
  r->authenticated = (f & 1) != 0;
  r->secure_connections = (f & 2) != 0;
  r->have_irk = (f & 4) != 0;
  r->have_local_csrk = (f & 8) != 0;
  r->have_peer_csrk = (f & 16) != 0;
  r->have_local_irk = (f & 32) != 0;
  r->candidate_present = (f & 64) != 0;
  r->candidate_verified = (f & 128) != 0;
  memcpy(r->local_csrk, in + p, 16);
  p += 16;
  memcpy(r->peer_csrk, in + p, 16);
  p += 16;
  memcpy(r->local_irk, in + p, 16);
  p += 16;
  memcpy(r->candidate_irk, in + p, 16);
  p += 16;
  memcpy(r->candidate_identity, in + p, 6);
  p += 6;
  r->candidate_type = in[p++];
  memcpy(r->local_ltk, in + p, 16);
  p += 16;
  memcpy(r->local_rand, in + p, 8);
  p += 8;
  r->local_ediv = (uint16_t)(in[p] | ((uint16_t)in[p + 1] << 8));
  p += 2;
  if (in[p] & ~3)
    return false;
  r->have_local_ltk = (in[p] & 1) != 0;
  r->have_peer_ltk = (in[p] & 2) != 0;
  return true;
}
static dbt_result_t read_record(nvs_handle_t h, const char *key, uint8_t type, const uint8_t id[6],
                                dbt_smp_bond_record_t *r) {
  uint8_t bytes[RECORD_BYTES];
  size_t n = sizeof(bytes);
  esp_err_t e = nvs_get_blob(h, key, bytes, &n);
  if (e == ESP_ERR_NVS_NOT_FOUND)
    return DBT_EMPTY;
  bool ok = e == ESP_OK && n == sizeof(bytes) && decode(bytes, r) && valid(r, type, id);
  wipe(bytes, sizeof(bytes));
  if (!ok) {
    wipe(r, sizeof(*r));
    return DBT_INVALID;
  }
  return DBT_OK;
}
static bool aes128(void *ctx,const uint8_t key[16],const uint8_t block[16],uint8_t out[16]) {
  (void)ctx; mbedtls_aes_context aes; mbedtls_aes_init(&aes);
  int rc=mbedtls_aes_setkey_enc(&aes,key,128);
  if (!rc) rc=mbedtls_aes_crypt_ecb(&aes,MBEDTLS_AES_ENCRYPT,block,out);
  mbedtls_aes_free(&aes); return rc==0;
}
bool dbt_smp_nvs_verify_rpa(const uint8_t rpa[6],const uint8_t irk[16],bool *matching) {
  return dbt_smp_ah_resolve(aes128,NULL,irk,rpa,matching);
}
static int hex_digit(char c) {
  if (c>='0' && c<='9') return c-'0';
  if (c>='a' && c<='f') return c-'a'+10;
  return -1;
}
static bool parse_bond_key(const char *key,uint8_t *type,uint8_t id[6]) {
  if (strlen(key)!=13 || (key[0]!='0' && key[0]!='1')) return false;
  *type=(uint8_t)(key[0]-'0');
  for (unsigned i=0;i<6;i++) {
    int a=hex_digit(key[1+i*2]),b=hex_digit(key[2+i*2]);
    if (a<0 || b<0) return false;
    id[i]=(uint8_t)((a<<4)|b);
  }
  return true;
}
void dbt_smp_nvs_init(dbt_smp_nvs_t *provider, uint32_t handle) {
  if (provider) {
    provider->handle = handle;
    provider->poisoned = false;
  }
}
/* Versioned transaction intent: magic, version, operation, identity type,
 * identity, before-present flag, SHA-256(before), SHA-256(after), SHA-256(header).
 * The hash detects accidental mismatch; it is not an authentication tag. */
static bool hash(const uint8_t *input,size_t n,uint8_t out[32]) {
  return mbedtls_sha256(input,n,out,0)==0;
}
static dbt_result_t read_intent(nvs_handle_t h,uint8_t marker[INTENT_BYTES]) {
  size_t n=INTENT_BYTES;
  esp_err_t e=nvs_get_blob(h,INTENT_KEY,marker,&n);
  if (e==ESP_ERR_NVS_NOT_FOUND) return DBT_EMPTY;
  uint8_t digest[32];
  bool ok=e==ESP_OK && n==INTENT_BYTES && !memcmp(marker,"DBTI",4) && marker[4]==1 &&
          (marker[5]==DBT_SMP_BOND_STORE || marker[5]==DBT_SMP_BOND_DELETE) && marker[6]<=1 &&
          marker[13]<=1 && hash(marker,78,digest) && !memcmp(marker+78,digest,32);
  if (ok && !marker[13])
    for (unsigned i=14;i<46;i++) if (marker[i]) ok=false;
  if (ok && marker[5]==DBT_SMP_BOND_DELETE)
    for (unsigned i=46;i<78;i++) if (marker[i]) ok=false;
  wipe(digest,sizeof(digest));
  if (!ok) { wipe(marker,INTENT_BYTES); return DBT_INVALID; }
  return DBT_OK;
}
static bool clear_intent(nvs_handle_t h) {
  esp_err_t e=nvs_erase_key(h,INTENT_KEY);
  if (e!=ESP_OK) return false;
  if (nvs_commit(h)!=ESP_OK) return false;
  size_t n=0;
  return nvs_get_blob(h,INTENT_KEY,NULL,&n)==ESP_ERR_NVS_NOT_FOUND;
}
dbt_result_t dbt_smp_nvs_resolve_rpa(dbt_smp_nvs_t *provider,const uint8_t rpa[6],
                                      dbt_smp_bond_record_t *out,bool *candidate) {
  if (!provider || provider->poisoned || !rpa || !out || !candidate || (rpa[5]&0xc0u)!=0x40u ||
      (rpa[3]==0 && rpa[4]==0 && (rpa[5]&0x3fu)==0) ||
      (rpa[3]==0xff && rpa[4]==0xff && (rpa[5]&0x3fu)==0x3f)) return DBT_INVALID;
  wipe(out,sizeof(*out)); *candidate=false;
  uint8_t marker[INTENT_BYTES],id[6],type;
  dbt_result_t rc=read_intent(provider->handle,marker); wipe(marker,sizeof(marker));
  if (rc!=DBT_EMPTY) {
    if (rc==DBT_INVALID) provider->poisoned=true;
    return rc==DBT_OK ? DBT_BUSY : DBT_INVALID;
  }
  nvs_iterator_t iterator=NULL;
  esp_err_t e=nvs_entry_find_in_handle(provider->handle,NVS_TYPE_ANY,&iterator);
  if (e==ESP_ERR_NVS_NOT_FOUND) return DBT_EMPTY;
  if (e!=ESP_OK) return DBT_INVALID;
  unsigned visited=0,matches=0; bool match_candidate=false;
  dbt_smp_bond_record_t record,found; memset(&found,0,sizeof(found));
  rc=DBT_EMPTY;
  while (e==ESP_OK) {
    if (++visited>32) { rc=DBT_NO_SPACE; break; }
    nvs_entry_info_t info; e=nvs_entry_info(iterator,&info);
    if (e!=ESP_OK) { rc=DBT_INVALID; break; }
    if (parse_bond_key(info.key,&type,id)) {
      if (info.type!=NVS_TYPE_BLOB || read_record(provider->handle,info.key,type,id,&record)!=DBT_OK) {
        rc=DBT_INVALID; break;
      }
      bool primary=false,alternative=false;
      if ((record.have_irk && !dbt_smp_ah_resolve(aes128,NULL,record.irk,rpa,&primary)) ||
          (record.candidate_present && !dbt_smp_ah_resolve(aes128,NULL,record.candidate_irk,rpa,&alternative))) {
        wipe(&record,sizeof(record)); rc=DBT_INVALID; break;
      }
      if (primary && alternative && (record.candidate_type!=record.identity_type ||
                                     memcmp(record.candidate_identity,record.identity,6) ||
                                     memcmp(record.irk,record.candidate_irk,16))) {
        wipe(&record,sizeof(record)); rc=DBT_INVALID; break;
      }
      if (primary || alternative) {
        if (++matches>1) { wipe(&record,sizeof(record)); rc=DBT_INVALID; break; }
        found=record; match_candidate=alternative;
      }
      wipe(&record,sizeof(record));
    }
    e=nvs_entry_next(&iterator);
  }
  nvs_release_iterator(iterator);
  if (rc==DBT_EMPTY && e!=ESP_ERR_NVS_NOT_FOUND) rc=DBT_INVALID;
  if (rc==DBT_EMPTY && matches==1) { *out=found; *candidate=match_candidate; rc=DBT_OK; }
  wipe(&record,sizeof(record)); wipe(&found,sizeof(found)); wipe(id,sizeof(id)); return rc;
}
dbt_result_t dbt_smp_nvs_pending(dbt_smp_nvs_t *provider,dbt_smp_bond_op_t *op,
                                  uint8_t *type,uint8_t id[6]) {
  if (!provider || provider->poisoned || !op || !type || !id) return DBT_INVALID;
  uint8_t marker[INTENT_BYTES];
  dbt_result_t rc=read_intent(provider->handle,marker);
  if (rc==DBT_OK) { *op=(dbt_smp_bond_op_t)marker[5]; *type=marker[6]; memcpy(id,marker+7,6); }
  else if (rc==DBT_INVALID) provider->poisoned=true;
  wipe(marker,sizeof(marker)); return rc;
}
dbt_result_t dbt_smp_nvs_reconcile(dbt_smp_nvs_t *provider,dbt_smp_bond_op_t op,
                                    uint8_t type,const uint8_t id[6],bool *applied) {
  if (!provider || provider->poisoned || !id || !applied || type>1 ||
      (op!=DBT_SMP_BOND_STORE && op!=DBT_SMP_BOND_DELETE)) return DBT_INVALID;
  uint8_t marker[INTENT_BYTES],bytes[RECORD_BYTES],digest[32];
  dbt_smp_bond_record_t record;
  dbt_result_t rc=read_intent(provider->handle,marker);
  if (rc==DBT_EMPTY) goto done;
  if (rc!=DBT_OK) { provider->poisoned=true; goto done; }
  if (marker[5]!=(uint8_t)op || marker[6]!=type || memcmp(marker+7,id,6)) { rc=DBT_INVALID; goto done; }
  char key[14]; make_key(key,type,id);
  rc=read_record(provider->handle,key,type,id,&record);
  if (rc!=DBT_OK && rc!=DBT_EMPTY) { rc=DBT_INVALID; goto done; }
  bool exists=rc==DBT_OK;
  if (exists) {
    encode(&record,bytes);
    if (!hash(bytes,sizeof(bytes),digest)) { rc=DBT_INVALID; goto done; }
  }
  bool before=exists ? marker[13] && !memcmp(digest,marker+14,32) : !marker[13];
  bool after=op==DBT_SMP_BOND_DELETE ? !exists : exists && !memcmp(digest,marker+46,32);
  if (!before && !after) { rc=DBT_BUSY; goto done; }
  if (!clear_intent(provider->handle)) { provider->poisoned=true; rc=DBT_INVALID; goto done; }
  *applied=after;
  rc=DBT_OK;
done:
  wipe(marker,sizeof(marker)); wipe(bytes,sizeof(bytes)); wipe(digest,sizeof(digest)); wipe(&record,sizeof(record));
  return rc;
}
dbt_result_t dbt_smp_nvs_execute(dbt_smp_nvs_t *provider, const dbt_smp_bond_job_t *j,
                                 dbt_smp_bond_record_t *loaded) {
  if (!provider || provider->poisoned || !j || !j->generation || !j->serial || j->identity_type > 1 ||
      j->op < DBT_SMP_BOND_STORE || j->op > DBT_SMP_BOND_DELETE)
    return DBT_INVALID;
  nvs_handle_t h = provider->handle;
  if (j->op == DBT_SMP_BOND_LOAD && !loaded)
    return DBT_INVALID;
  if (j->op == DBT_SMP_BOND_LOAD) wipe(loaded,sizeof(*loaded));
  uint8_t intent[INTENT_BYTES];
  dbt_result_t intent_rc=read_intent(h,intent);
  if (intent_rc!=DBT_EMPTY) {
    wipe(intent,sizeof(intent));
    if (intent_rc==DBT_INVALID) provider->poisoned=true;
    return intent_rc==DBT_OK ? DBT_BUSY : DBT_INVALID;
  }
  wipe(intent,sizeof(intent));
  char key[14];
  make_key(key, j->identity_type, j->identity);
  if (j->op == DBT_SMP_BOND_LOAD) {
    return read_record(h, key, j->identity_type, j->identity, loaded);
  }
  dbt_smp_bond_record_t old;
  memset(&old, 0, sizeof(old));
  dbt_result_t old_rc = read_record(h, key, j->identity_type, j->identity, &old);
  if (old_rc != DBT_OK && old_rc != DBT_EMPTY)
    return DBT_INVALID;
  uint8_t bytes[RECORD_BYTES],check[RECORD_BYTES],marker[INTENT_BYTES];
  memset(bytes,0,sizeof(bytes)); memset(check,0,sizeof(check)); memset(marker,0,sizeof(marker));
  if (old_rc==DBT_OK) { encode(&old,check); marker[13]=1; }
  if (j->op == DBT_SMP_BOND_STORE) {
    if (!valid(&j->record, j->identity_type, j->identity)) {
      wipe(&old, sizeof(old));
      return DBT_INVALID;
    }
    if (old_rc == DBT_OK &&
        ((old.authenticated && !j->record.authenticated) ||
         (old.secure_connections && !j->record.secure_connections) || old.key_size > j->record.key_size ||
         (old.have_irk && (!j->record.have_irk || memcmp(old.irk,j->record.irk,16))) ||
         (!old.have_irk && j->record.have_irk &&
          (!old.candidate_present || memcmp(old.candidate_irk,j->record.irk,16))) ||
         (!old.candidate_present && j->record.candidate_present &&
          (!old.have_irk || memcmp(old.irk,j->record.candidate_irk,16))) ||
         (old.candidate_present &&
          (!j->record.candidate_present || (old.candidate_verified && !j->record.candidate_verified) ||
           old.candidate_type != j->record.candidate_type ||
           memcmp(old.candidate_identity, j->record.candidate_identity, 6) ||
           memcmp(old.candidate_irk, j->record.candidate_irk, 16))))) {
      wipe(&old, sizeof(old));
      return DBT_UNSUPPORTED;
    }
    encode(&j->record, bytes);
  }
  wipe(&old, sizeof(old));
  memcpy(marker,"DBTI",4); marker[4]=1; marker[5]=(uint8_t)j->op; marker[6]=j->identity_type;
  memcpy(marker+7,j->identity,6);
  bool ok=(!marker[13] || hash(check,sizeof(check),marker+14)) &&
          (j->op!=DBT_SMP_BOND_STORE || hash(bytes,sizeof(bytes),marker+46)) && hash(marker,78,marker+78);
  if (!ok) goto finish;
  esp_err_t e=nvs_set_blob(h,INTENT_KEY,marker,sizeof(marker));
  if (e==ESP_OK) e=nvs_commit(h);
  uint8_t readback[INTENT_BYTES]; size_t n=sizeof(readback);
  if (e==ESP_OK) e=nvs_get_blob(h,INTENT_KEY,readback,&n);
  ok=e==ESP_OK && n==sizeof(readback) && !memcmp(marker,readback,sizeof(marker));
  wipe(readback,sizeof(readback));
  if (!ok) goto finish;
  if (j->op==DBT_SMP_BOND_STORE) e=nvs_set_blob(h,key,bytes,sizeof(bytes));
  else { e=nvs_erase_key(h,key); if (e==ESP_ERR_NVS_NOT_FOUND) e=ESP_OK; }
  if (e==ESP_OK) e=nvs_commit(h);
  n=j->op==DBT_SMP_BOND_STORE ? sizeof(check) : 0;
  if (e==ESP_OK) e=nvs_get_blob(h,key,j->op==DBT_SMP_BOND_STORE ? check : NULL,&n);
  ok=j->op==DBT_SMP_BOND_STORE ? e==ESP_OK && n==sizeof(check) && !memcmp(bytes,check,sizeof(bytes))
                                : e==ESP_ERR_NVS_NOT_FOUND;
  if (ok) ok=clear_intent(h);
finish:
  wipe(bytes,sizeof(bytes)); wipe(check,sizeof(check)); wipe(marker,sizeof(marker));
  if (!ok) provider->poisoned=true;
  return ok ? DBT_OK : DBT_INVALID;
}
