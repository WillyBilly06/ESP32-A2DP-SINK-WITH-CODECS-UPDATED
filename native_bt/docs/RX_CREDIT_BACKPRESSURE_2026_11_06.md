# Downstream receive-credit backpressure — 2026-11-06

Status: source/build/readback/cold and two focused maximum-quality LDAC plus acknowledged BLE EQ paths verified. Packet drops were prevented in those runs; nonzero DMA-refill underruns remain. Full all-codec/security/lifecycle/OTA/stress/acoustic qualification is incomplete.

## Actual capacities and defect

The H4 FIFO is4096 bytes with520 bytes of event reserve. Its existing advertised controller-to-host ACL window was already3 packets at1024 bytes, as confirmed in516147fc boot logs. It was not15 packets or larger than the decoder queue; do not confuse this FIFO with16384 bytes of retained L2CAP reassembly storage. The new four-packet upper bound leaves the current effective window at3.

The decoder queue remains1800-byte payload x8 slots (1816-byte stride,14528 retained bytes). The previous runtime marked an HCI packet complete after its borrowed callback returned and immediately replenished receive credits. This protected the raw FIFO but did not backpressure downstream asynchronous media storage. At explicitly selected LDAC990kbps/96k/S32, image153107a5 recorded four real NO_SPACE drops at sequences3569,4246,4365,4529; incoming RTP sequence accounting was contiguous, and the worker reset four times after those host-created gaps. The payloads were661 bytes and fit their slots. This is a capacity/lifetime problem rather than an oversized-packet explanation.

## Policy and ownership

The optional owner-only completion gate is configured before RX flow starts or any link is present. It applies only to the live Classic A2DP signaling handle/generation while streaming; BLE, other links and teardown remain eligible. Classic completions are retained when at least4 encoded slots are occupied. At queue3 with the actual3-packet window, at most6 slots can be occupied by already authorized incoming packets; the conservative four-packet upper bound would allow at most7, within8.

The Host Number Of Completed Packets command still bypasses ordinary command credits and retains its TX bytes through transport backpressure. Poll clears only the entries actually included in a submitted command, preserving withheld generations. Disconnect retires matching entries. The wait query skips held completions instead of returning zero and spinning. Both deferred and direct codec consumers notify the host on the downward half-capacity crossing, so credit release is event-driven. No buffer, task, codec quality reduction or new periodic media poll is introduced.

The selected BTstack reference9edb5d82 sections inspected were src/hci.c around1289–1296,6611–6653 and8518–8523: per-connection completions, manual0x0c35 without normal command credit, transport-buffer ownership and scheduler priority. The downstream queue gate is an independently authored adaptation for this asynchronous worker architecture, not imported BTstack code or a claim of whole-stack comparison completion.

## Hardware evidence

Current image b6b6c2633169812fa0cd0aed9e143cd2125fb15359ced18db39d9369d36d9210, archive runs/1793978223319806700-b6b6c2633169. Ccache build,10349 stable source hashes, full app/selector readbacks(921600 write/115200 read), protected original/bootloader/table unchanged, owned VALID/no fallback and37 existing RF=false cold codec checks pass.

Fresh phone Developer UI confirmed LDAC96k/32-bit/stereo and Optimized for Audio Quality after reconnect; reconnect resets quality to adaptive, so earlier low-payload traces must not be labeled maximum-rate. Two12s finite exact-address sources with actual BDK Deep Bass then Balanced commands both ended normally,0 Android source underrun/focus loss/route loss. Both EQ changes received Speaker accepted EQ; Balanced was restored.

| Focused maximum-quality run | Payload over sampled same-epoch interval | Queue drops / decoder rejects / resets | Queue high-water | Native refill underrun frames | Flushed accepted=rendered=DMA-source |
|---|---:|---:|---:|---:|---:|
| ldac990-eq-first-1793978672934 |968.9kbit/s|0/0/0|6/8|256|1437696|
| ldac990-eq-repeat-1793978735609 |981.5kbit/s|0/0/0|6/8|2048|1434624|

The gate retained up to3 Classic completions and resumed them; ordinary snapshots also showed zero pending BLE completions after ACKs. No RTP input sequence break, descriptor fault or watchdog was recorded. These varying short traces do not establish an isolated speedup, sustained throughput guarantee or complete BLE latency proof. Worker trace-ring loss is reported separately; host drop records and aggregate/frozen snapshots were retained.

## Remaining DMA-refill issue

The repeat first shortage had256 decoded input frames,0 ready output, decoder waiting for renderer and renderer active54us when the ISR chose silence. The first run's decoder had begun47us before the shortage. This narrows the next investigation beyond encoded queue loss.

The current on_sent callback reads output immediately at EOF, fills shortage with zero and never revisits that completed buffer. Actual IDF6.1 esp_driver_i2s/i2s_common.c:925–947 obtains the completed descriptor from the hardware EOF address, calls on_sent, then queues that completed buffer. Its i2s_channel_write path acquires completed buffers and writes after the callback. The working Bluedroid renderer uses that later writer path. EOF is therefore a buffer-release event, not inherently the last safe refill instant. A safe asynchronous refill lease must prove ordering, buffer lifetime and hardware reuse deadlines; merely renaming counters or writing old buffers late is not a fix. No late-refill implementation is included in this checkpoint.

Private byte-preserving evidence lives under native_bt/_build/experiments/shared-render-handoff-1793975538770/rx-backpressure. Older completed UART copies were gzip-compressed after verifying exact round-trip bytes, with SHA receipts in trace-compression-1793978103943.json; hardware archives/live monitor were untouched. Decompress first for readers requiring plain NDJSON.
