# A2DP stream-boundary observations — 2026-11-08

This stage adds bounded lifecycle diagnostics and removes mutable selected-queue depth reads from the background logger. It does not change audio format, codec math, buffer capacity, startup guard, controller credits, task priorities or affinity.

The internal host DTO now uses seven values within the existing32-byte trace payload. Static assertions preserve the44-byte slot and16 slots per producer. Host submission/state records use ring0; decoder handoffs use ring1 only after task and current-job identity checks. No renderer task writes either ring. Full rings increment their existing loss count and return without waiting, allocating or calling the clock.

Records remain separate:

- a2dp_worker_submit: accepted binding/token/operation, observed gate epoch and started epoch, submission begin and event observation times. Gate closure falls within that interval. It is not an HCI/RF arrival timestamp.
- a2dp_lifecycle: final protocol event/state plus binding, pending token, media binding and observed epochs. It is not automatically the time the peer sent Suspend.
- a2dp_stream_handoff: current immutable job binding/token/epoch, begin/poll/cancel phase, start/stop, renderer ticket and provider result. It observes the decoder's call return, not the precise DMA-disable instruction or a peer acknowledgment.

Background formatting and UART remain on the existing low-priority logger at921600 baud. Pending/BUSY polls do not produce repeated normal records. Typical added work is a few records per control operation, never per audio packet. Independent event timestamps must be used instead of UART receive order or periodic snapshot time.

The decoder_input_queue record now reads physical depth and stride from one atomic queue_layout word; it reports zero reserved queue bytes when that published working set is absent. It does not read a mutable decoder-owned depth field during reconfiguration.

Six actual app variants pass source-matched checks, including ownership rejection with a live foreign job, empty-job rejection, full-ring behavior, immutable identity/value copies, accepted-versus-BUSY submission, exact callback wiring and the real background formatter. Frozen trace/formatter checks and their negative controls pass. Existing duration/framing/delay checks pass with both negative controls. Some runs were held only by the existing artifact-size gate; completed objects/binaries/source archives were consolidated with provenance and hash verification, without changing the512MiB gate or discarding failed receipts.

The ccache target build,10,424-input source check, offline image admission and audio ISR audit pass for image9048cff45172f4d574a9b6485a89c2d0fee40864a5518bf622ee86765a0fa566. Relative to3b246522d523, linked DRAM data/BSS and IRAM are unchanged; flash text grows420 bytes. These are source/build checks, not continuity qualification.

## Device observation

The image passed COM6 readback, preserved the original application and kept automatic Bluedroid restoration disabled. Deployment archive: runs/1794168084738234900-9048cff45172. The existing37 cold configurations /173 callbacks passed; tracked peak56,899B, allocation minimum106,144B, decoder-stack low-water3,484B and maximum cold decode12,573us. No boot watchdog/backtrace/idle-starvation markers appeared.

Matched native YouTube excerpts used LHDC192k/S24/stereo/fixed900, requested10s playback, independent14s pause guards and verified I2S spans below15s. Metronome had0 finalized unfilled frames. Music had1,792 initial shortage,832 recovered and960 finalized unfilled frames. Its first finalized gap was before the phone pause request,165,576us after the Streaming event and13,074,890us before the accepted Suspend call began. Thus the gap cannot be attributed to that later renderer shutdown.

At the music's initial shortage both PCM queues were empty and the decoder was still in decode. One millisecond later,768 raw frames and64 processed frames existed; a partial recovery had missed the final deadline. The last correlated media HCI callback was5,131us before that finalized gap, with one ACL-metadata item pending. The latest coherent receive-credit sample showed no held completions, but was2,330us old. These observations do not identify an RF loss or controller acknowledgment.

Between the first adjacent-packet clock record and the gap's last media-admission begin, host time advanced123,379us while RTP advanced75,000us: approximately48,379us excess. The observation points differ slightly, so this is a cadence estimate rather than an exact RF interval. The applied startup guard was50,305us with only four encoded packets at enable. A bounded80ms reserve trial, adding30ms startup latency without a new buffer, is the next experiment; it is not yet a demonstrated fix.

The clean metronome's accepted stop interval and decoder handoff show about26ms from renderer request to successful poll observation, followed by final Open state. That is not proof that DMA continued running for26ms: the record timestamps observe call returns, not the precise disable instruction.

Sustained1Mbps plus BLE/control, acoustic quality and complete feature/security/OTA qualification remain open.

Artifacts: native_bt/_build/experiments/stream-boundary-1794167102404. Completed source-input ZIPs are preserved byte-for-byte in the tar.xz archive and its per-member SHA manifest; historical executable/log archives and typed-object retirement manifests are separate. Codec algorithms and portable core are unchanged from the packed-input checkpoint.
