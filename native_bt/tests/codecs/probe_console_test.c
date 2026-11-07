#include <stdio.h>
static unsigned lock_depth,lock_errors;
void console_test_flockfile(FILE *file){
    if((lock_depth==0 && file!=stdout)||(lock_depth==1 && file!=stderr)||lock_depth>=2)++lock_errors;
    ++lock_depth;
}
void console_test_funlockfile(FILE *file){
    if(!lock_depth || (lock_depth==2 && file!=stderr)||(lock_depth==1 && file!=stdout))++lock_errors;
    if(lock_depth)--lock_depth;
}
#define flockfile console_test_flockfile
#define funlockfile console_test_funlockfile
#define CONFIG_ESP_CONSOLE_UART 1
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#if CONSOLE_TEST_LINE_MODE==0
#define CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF 1
#elif CONSOLE_TEST_LINE_MODE==1
#define CONFIG_LIBC_STDOUT_LINE_ENDING_LF 1
#else
#define CONFIG_LIBC_STDOUT_LINE_ENDING_CR 1
#endif
#include "../../esp32_probe/main/probe_console.c"
#include <string.h>
#include <stdio.h>
#define CHECK(x) do{if(!(x)){printf("FAIL console line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static unsigned writes;static char wire[2048];static size_t wire_bytes;static bool short_write;
int uart_write_bytes(uart_port_t port,const void *data,size_t length){
    if(port || length>sizeof wire || lock_depth!=2)return -1;
    memcpy(wire,data,length);wire_bytes=length;++writes;return (int)length-(short_write?1:0);
}
static bool installed;static unsigned installs,switches,errors;static esp_err_t result;
bool uart_is_driver_installed(uart_port_t port){if(port!=0)++errors;return installed;}
esp_err_t uart_driver_install(uart_port_t port,int rx,int tx,int depth,void *queue,int flags){
    if(port || rx!=256 || tx || depth || queue || flags)++errors;
    ++installs;if(result==ESP_OK)installed=true;return result;
}
void uart_vfs_dev_use_driver(int port){if(port || !installed)++errors;++switches;}
int main(void){
    bool created=false;CHECK(probe_console_irq_init(&created)==ESP_OK && created && installs==1 && switches==1 && !errors);
    created=true;CHECK(probe_console_irq_init(&created)==ESP_OK && !created && installs==1 && switches==2 && !errors);
    installed=false;result=ESP_ERR_NO_MEM;created=true;
    CHECK(probe_console_irq_init(&created)==ESP_ERR_NO_MEM && !created && installs==2 && switches==2 && !errors);
    result=ESP_OK;CHECK(probe_console_irq_init(NULL)==ESP_OK && installs==3 && switches==3 && !errors);
    char line[16]="abc\n";
    CHECK(probe_console_write_record(line,4,sizeof line) && writes==1);
#if CONSOLE_TEST_LINE_MODE==0
    CHECK(wire_bytes==5 && !memcmp(wire,"abc\r\n",5));
    char tight[2]={'a','\n'};CHECK(!probe_console_write_record(tight,2,2) && writes==1);
#elif CONSOLE_TEST_LINE_MODE==1
    CHECK(wire_bytes==4 && !memcmp(wire,"abc\n",4));
#else
    CHECK(wire_bytes==4 && !memcmp(wire,"abc\r",4));
#endif
    char bad[8]="a\nb\n";CHECK(!probe_console_write_record(bad,4,sizeof bad));
    CHECK(!probe_console_write_record(NULL,1,1) && probe_console_write_record(NULL,0,0));
    CHECK(!probe_console_write_record(line,17,16));
    char long_line[1025]={0};long_line[1024]='\n';CHECK(!probe_console_write_record(long_line,sizeof long_line,sizeof long_line));
    short_write=true;char fail[8]="abc\n";CHECK(!probe_console_write_record(fail,4,sizeof fail));
    CHECK(!lock_depth && !lock_errors);
    puts("PASS bounded console records: one byte-exact driver call, configured newline, full/short/invalid records");
    puts("PASS console IRQ: existing console only, success/reuse/failure/NULL,256-byte RX, no TX/event queue, no pin/baud change or driver deletion");return 0;
}
