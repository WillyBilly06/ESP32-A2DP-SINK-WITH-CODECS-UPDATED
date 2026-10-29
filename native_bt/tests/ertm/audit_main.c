#include <stdio.h>
int audit_startup_window_matrix(void); int audit_duplicate_multiround(void);
int audit_bidirectional_loss_all_windows(void); int audit_busy_poll_final(void);
int audit_gap_across_sequence_wrap(void); int audit_coalesced_final_causes_timeout(void); int audit_pinned_ack_receive_progress(void); int audit_bad_fcs_pinned_loan(void); int audit_infinite_retransmit_counter(void); int audit_seeded_loss_and_busy(void); int audit_pinned_teardown_matrix(void); int audit_rej_poll_busy_window_matrix(void); int audit_repeated_loss_finite_recovery(void); int audit_repeated_poll_pins(void); int audit_retry_limit_during_other_pin(void); int audit_timer_wrap_and_exhaustion(void);
unsigned audit_window(void); unsigned audit_case(void); unsigned audit_step(void); unsigned audit_a(void); unsigned audit_b(void);
int main(void) {
    struct {const char *name; int (*run)(void);} tests[]={
        {"startup_window_matrix",audit_startup_window_matrix},{"duplicate_multiround",audit_duplicate_multiround},
        {"bidirectional_loss_all_windows",audit_bidirectional_loss_all_windows},{"busy_poll_final",audit_busy_poll_final},{"rej_poll_busy_window_matrix",audit_rej_poll_busy_window_matrix},
        {"repeated_loss_finite_recovery",audit_repeated_loss_finite_recovery},{"repeated_poll_pins",audit_repeated_poll_pins},{"gap_across_sequence_wrap",audit_gap_across_sequence_wrap},{"coalesced_final_causes_timeout",audit_coalesced_final_causes_timeout},{"pinned_ack_receive_progress",audit_pinned_ack_receive_progress},{"bad_fcs_pinned_loan",audit_bad_fcs_pinned_loan},{"infinite_retransmit_counter",audit_infinite_retransmit_counter},{"seeded_loss_and_busy",audit_seeded_loss_and_busy},{"pinned_teardown_matrix",audit_pinned_teardown_matrix},{"retry_limit_during_other_pin",audit_retry_limit_during_other_pin},
        {"timer_wrap_and_exhaustion",audit_timer_wrap_and_exhaustion}};
    unsigned failures=0;
    for(unsigned i=0;i<sizeof(tests)/sizeof(tests[0]);i++) {
        int line=tests[i].run(); if(line) {failures++;printf("FAIL %s line %d window %u case %u step %u a %u b %u\n",tests[i].name,line,audit_window(),audit_case(),audit_step(),audit_a(),audit_b());}
        else printf("PASS %s\n",tests[i].name);
    }
    return failures ? 1 : 0;
}
