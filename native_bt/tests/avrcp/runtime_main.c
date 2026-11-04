#include <stdio.h>
int avrcp_runtime_interleave_credit(void);
int avrcp_runtime_disconnect_reuse(void);
int avrcp_runtime_competing_loan(void);
int main(void) {
    int r=avrcp_runtime_interleave_credit(); if(r){fprintf(stderr,"FAIL interleave_credit line %d\n",r);return 1;}
    puts("PASS interleave_credit");
    r=avrcp_runtime_disconnect_reuse(); if(r){fprintf(stderr,"FAIL disconnect_reuse line %d\n",r);return 1;}
    puts("PASS disconnect_reuse");
    r=avrcp_runtime_competing_loan(); if(r){fprintf(stderr,"FAIL competing_loan line %d\n",r);return 1;}
    puts("PASS competing_loan"); return 0;
}
