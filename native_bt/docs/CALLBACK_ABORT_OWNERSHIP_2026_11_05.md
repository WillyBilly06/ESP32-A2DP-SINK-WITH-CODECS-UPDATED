# Callback-safe host and runtime abort

Source comparison uses BTstack commit 9edb5d82ea6ec4466a393415d65030929330acdf as a reference only. No reference implementation is integrated.

BTstack l2cap.c:1297–1324 emits explicit channel/buffer lifetime notifications; :5296–5301 marks closed, notifies, then removes/frees the channel. hci.c:1385–1403 stops its connection timer, removes/frees the connection, then emits the connection-count change. These are distinct notification contracts, not interchangeable cleanup calls. The owned API borrows channel, packet and return-parameter views through callback return.

## Defects in the owned boundaries

- Runtime abort immediately cleared RX storage, even when called from a borrowed data callback. That could invalidate the callback's bytes before return.
- Runtime set cleared before calling channels_clear. The manager refuses callback-time clear; the early flag could prevent a later cleanup, leaving channel resources retained.
- Host failure could synchronously nest aborted-command callbacks and wipe RX bytes when cancellation was initiated outside host polling. Runtime delegated policy/allocator/TX callbacks did not all fence recursive polling or cleanup.
- Runtime checked host failure before channel polling, but not after channel callbacks or all TX completion paths. A later failure could still be followed by ACL submission, or cleanup could wait forever with no new ingress.

## Change and ownership

Host failure records terminal state and ingress fault immediately. Command notifications and observers carry an owner-only borrowed-callback fence; abort drains remaining commands and wipes RX only after poll/callback unwind. Cancellation removes its exact command before notification, preserving normal follow-up enqueue on a healthy host. Existing HCI timeout outcomes and first-fault policy remain distinct.

Runtime reuses its existing owner-operation flag for delegated allocation/release, policy, channel/data/link/packet/SCO and TX completion callbacks. Nested callbacks preserve/restore the previous flag. TX completion frees the slot before notification and never touches it afterward, so healthy callbacks can still enqueue their next SDU. No new task, queue, lock, buffer or polling interval is added.

Runtime abort marks failure/ready=false immediately and stops RX flow accounting. In a borrowed callback/owner operation it defers command draining, RX/channel destruction and SDU loan returns. At a safe owner boundary, a single finalizer performs cleanup with callback reentry fenced. Channel clear now runs after its callback restriction has ended; remaining ERTM descriptors are retired too. Outside callbacks, abort still finalizes synchronously. Transport-retained packet/command storage remains valid until can_send returns it; abort does not authorize reinitialization while that loan exists.

The owner checks failure after channel polling, before transmission and at the end of TX work. Failure inside a local completion cannot send to the next link. Failed-but-not-cleared state returns wait_ms=0 for the cleanup turn, then the empty failed owner can sleep. Work is still bounded by existing queue/link/RX/TX limits; no credit-wait spin is added.

## Verification scope

Both authored host/runtime bodies and contracts were read, including their current command, callback, scheduler, deadline and loan boundaries. Channel/provider/ERTM internals remain only partially reviewed. This is not complete layer or feature parity.

Strict portable Clang analysis reports zero diagnostics for host/runtime. Matched standalone Xtensa -Os compilation with warnings as errors passes before/after. Target layout is unchanged: host 6696 bytes, runtime 11160 bytes; checked old field offsets are identical. The new bool occupies existing padding. Host poll own frame remains 80 bytes, runtime poll 96, finish_tx 32 and runtime abort 32. Host fail grows 32→64; the separate finalizer is 48 bytes. These are individual compiler frames, not whole-chain/task stack headroom or execution-time measurements.

The full ccache product build passes with all 10348 source fingerprints stable and rebuildable storage 455.611 MiB under the 512 MiB cap. Linked text grows 632 bytes to 883443; BSS 50412, IRAM 122255 and DRAM data 23893 are unchanged. COM6 upload at 921600 and image/selector readback at 230400 match. The original application full-region MD5 is unchanged; owned ota1 remains VALID/kept_active with automatic Bluedroid restoration disabled. Ordinary startup is ready with no watchdog/backtrace, all 37 existing cold codec checks pass (RF false, peak56899), and idle heap remains 137912 free/67356 regional minimum/110592 largest. UART source129 is connected. These observations do not exercise callback-time abort. Archive: native_bt/_build/hardware/runs/1793950513171106300-ed8ac589f89e/. No separate callback, allocator failure, packet, pairing, playback, RF, app, stress or fault campaign is run under the user's hold. Correct source lifetime and compilation do not establish live interoperability, underrun removal, throughput or LHDC real-time headroom.
