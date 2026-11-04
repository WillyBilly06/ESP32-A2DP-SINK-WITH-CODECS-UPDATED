#include <stdio.h>
int probe_runtime_exchange(void);
int main(void) {
    int line=probe_runtime_exchange();
    if(line) {printf("FAIL paired runtime line %d\n",line);return 1;}
    puts("PASS paired runtime exchange and pinned disconnect");return 0;
}
