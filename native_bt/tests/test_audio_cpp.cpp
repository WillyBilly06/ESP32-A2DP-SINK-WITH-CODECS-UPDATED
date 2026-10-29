#include "dsp_processor.h"
#include "nvs_settings.h"
#include <atomic>
#include <map>
#include <string>
#include <thread>
#include <vector>
#include <cstdio>
#include <cstring>

static std::map<std::string, uint32_t> flash_values, pending_values;
static std::vector<std::string> calls;
static bool fail_set, fail_commit;
extern "C" {
esp_err_t nvs_open(const char *, int, nvs_handle_t *handle) { calls.push_back("open"); *handle=1; return ESP_OK; }
void nvs_close(nvs_handle_t) { calls.push_back("close"); pending_values.clear(); }
esp_err_t nvs_commit(nvs_handle_t) {
    calls.push_back("commit");
    if (fail_commit) return ESP_FAIL;
    for (const auto &entry:pending_values) flash_values[entry.first]=entry.second;
    return ESP_OK;
}
esp_err_t nvs_set_u32(nvs_handle_t, const char *key, uint32_t value) {
    calls.push_back("set_u32");
    if (fail_set) return ESP_FAIL;
    pending_values[key]=value;
    return ESP_OK;
}
esp_err_t nvs_get_u32(nvs_handle_t, const char *key, uint32_t *value) {
    auto entry=flash_values.find(key);
    if (entry==flash_values.end()) return ESP_ERR_NVS_NOT_FOUND;
    *value=entry->second; return ESP_OK;
}
esp_err_t nvs_get_i8(nvs_handle_t h,const char *key,int8_t *value) {uint32_t v=0; auto e=nvs_get_u32(h,key,&v); *value=(int8_t)v; return e;}
esp_err_t nvs_get_u8(nvs_handle_t h,const char *key,uint8_t *value) {uint32_t v=0; auto e=nvs_get_u32(h,key,&v); *value=(uint8_t)v; return e;}
esp_err_t nvs_get_str(nvs_handle_t,const char *,char *,size_t *) {return ESP_ERR_NVS_NOT_FOUND;}
esp_err_t nvs_set_u8(nvs_handle_t,const char *,uint8_t) {return ESP_OK;}
esp_err_t nvs_set_str(nvs_handle_t,const char *,const char *) {return ESP_OK;}
const char *esp_err_to_name(esp_err_t) {return "mock";}
}
#define CHECK(expression) do { if (!(expression)) { std::fprintf(stderr,"line %d: %s\n",__LINE__,#expression); return 1; } } while(0)

static int nvs_tests() {
    flash_values[NVS_KEY_EQ_BASS]=(uint8_t)-4;
    flash_values[NVS_KEY_EQ_MID]=6;
    flash_values[NVS_KEY_EQ_TREB]=12;
    NVSSettings settings;
    CHECK(settings.load());
    int8_t b=0,m=0,t=0;
    settings.getEQ(b,m,t); CHECK(b==-4 && m==6 && t==12);
    calls.clear();
    CHECK(settings.saveEQ(12,-12,7));
    CHECK(calls==std::vector<std::string>({"open","set_u32","commit","close"}));
    CHECK(flash_values[NVS_KEY_EQ_PACKED]==0x0107f40c);
    NVSSettings restarted;
    CHECK(restarted.load()); restarted.getEQ(b,m,t); CHECK(b==12 && m==-12 && t==7);
    fail_commit=true;
    CHECK(!settings.saveEQ(-6,4,-2));
    settings.getEQ(b,m,t); CHECK(b==12 && m==-12 && t==7);
    fail_commit=false; fail_set=true; calls.clear();
    CHECK(!settings.saveEQ(-6,4,-2));
    CHECK(calls==std::vector<std::string>({"open","set_u32","close"}));
    fail_set=false;
    return 0;
}

static int concurrency_test() {
    DSPProcessor processor;
    processor.init(48000);
    std::atomic<bool> start{false};
    std::thread writer([&] {
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (unsigned i=0;i<30000;++i) {
            processor.setEQ((int8_t)((int)(i%25)-12),(int8_t)((int)((i/3)%25)-12),
                            (int8_t)((int)((i/7)%25)-12),48000);
            processor.setVolume((uint8_t)(i%128));
            processor.applyControlByte((uint8_t)(i%8));
            if (i%113==0) processor.zeroLevels();
            if (i%17==0) std::this_thread::yield();
        }
    });
    std::thread observer([&] {
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (unsigned i=0;i<30000;++i) {
            (void)processor.getPeakLin(i%3); (void)processor.getDB(i%3);
            (void)processor.getVolume();
            if(i%19==0) std::this_thread::yield();
        }
    });
    start.store(true,std::memory_order_release);
    for (unsigned block=0;block<12000;++block) {
        if (!processor.beginAudioBlock()) std::abort();
        for(unsigned i=0;i<32;++i) {
            int32_t left=(i&1)?(1<<27):-(1<<27),right=left;
            processor.processStereoQ31(left,right);
        }
    }
    writer.join(); observer.join();
    processor.setVolume(0);
    processor.beginAudioBlock();
    int32_t left=0,right=0;
    for(unsigned i=0;i<2000;++i) { left=right=INT32_MAX; processor.processStereoQ31(left,right); }
    CHECK(left==0 && right==0);
    return 0;
}

int main() {
    if(nvs_tests() || concurrency_test()) return 1;
    std::puts("PASS immediate NVS tuple, legacy migration, write/commit failures, concurrent controls/render/meter");
    return 0;
}
