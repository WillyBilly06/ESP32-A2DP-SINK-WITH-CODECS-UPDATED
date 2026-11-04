# CODEX MASTER GOAL: Build a Fully Custom, Ultra-Optimized, Asynchronous Bluetooth Stack for ESP32

## 1. Primary Objective

Design, implement, optimize, and integrate our own production-quality, extremely lightweight, high-performance Bluetooth host stack, written specifically for the original ESP32 architecture.

**This is a complete Bluetooth host-stack engineering project, not a wrapper around BTstack, Bluedroid, or NimBLE.**

The new implementation must support Bluetooth Classic A2DP, AVRCP, and Bluetooth Low Energy simultaneously, communicate directly with the ESP32 Bluetooth controller using HCI/VHCI, and preserve every feature currently available in our firmware within the scope of A2DP, BLE, audio processing, and application functionality.

The entire implementation must be:

- Fully asynchronous and event-driven.
- Non-blocking throughout all runtime-critical operations.
- Free of busy-waiting, polling spins, unbounded processing loops, and scheduling stalls.
- Extremely efficient in internal SRAM consumption.
- Optimized specifically for dual-core Xtensa LX6 at 240 MHz.
- Compatible with ESP-IDF 6.1.
- Capable of reliable, uninterrupted, full-quality audio playback.
- Designed with deterministic memory usage, bounded execution times, and explicit resource ownership.
- Robust under simultaneous high-throughput Bluetooth Classic audio and BLE workloads.

**The ultimate goal is maximum performance and minimum RAM usage without sacrificing a single existing A2DP, BLE, audio, security, or application feature.**

## 2. Hardware and Environment

Target:

- SoC: ESP32-D0WD-V3, revision 3.1.
- CPU: Dual-core Xtensa LX6, 240 MHz.
- Flash: 8 MB.
- PSRAM: None.
- Framework: ESP-IDF 6.1.
- Audio output: PCM5102A through I2S.
- Bluetooth: Classic BR/EDR and BLE 4.2.
- Operating system: ESP-IDF FreeRTOS.

Preserve the current firmware's functional behavior, hardware interfaces, peripheral configuration, and working components within the scope of this project.

The limited internal RAM is a fundamental design constraint. The system must coexist with the audio codecs, DSP, I2S DMA, BLE services, configuration storage, LEDs, user controls, and all other application functionality.

Do not assume additional RAM or a more powerful processor is available.

## 3. Build Our Own Bluetooth Host Stack

Replace the existing Bluetooth host dependency with our own modular, independently maintained implementation.

Retain Espressif's hardware Bluetooth controller and communicate through its documented HCI/VHCI interface.

Implement the host-side functionality required by the project, including:

### HCI / VHCI Transport

- Asynchronous HCI command submission.
- HCI event handling.
- ACL packet transmission and reception.
- Command and ACL flow control.
- Controller buffer and credit accounting.
- Packet fragmentation and reassembly.
- Connection creation and termination.
- Disconnect and error recovery.
- Controller readiness notifications.
- Event-driven transmit scheduling.
- Bounded queues and packet ownership.
- Safe handling of controller backpressure.

Never spin while waiting for controller availability or credits.

Respect the ESP-IDF VHCI calling requirements, including checking send availability and avoiding controller submissions from prohibited contexts.

### Bluetooth Classic

Implement and preserve all required Bluetooth Classic host functionality:

- GAP discovery and device management.
- Inquiry and paging behavior.
- Pairing, authentication, and bonding.
- Secure Simple Pairing and persistent link keys.
- SDP service registration and discovery.
- L2CAP signaling and data transport.
- Required L2CAP modes, including ERTM where existing features depend on it.
- AVDTP signaling and media transport.
- A2DP capability negotiation.
- Codec configuration and reconfiguration.
- Stream establishment, start, suspend, close, and recovery.
- AVRCP controller and target functions.
- Playback, pause, stop, previous, and next controls.
- Metadata and volume synchronization.
- Automatic reconnection.
- Connection state management.
- Reliable handling of simultaneous Bluetooth activities.

Preserve any additional existing A2DP/AVRCP features, including browsing, delay reporting, or other supported profile behavior.

Support A2DP Sink and any existing A2DP Source functionality without silently removing either role.

### Bluetooth Low Energy

Implement the necessary BLE host functionality directly rather than depending on a separate full-size Bluetooth host stack.

Preserve:

- BLE advertising and connections.
- GAP operations.
- ATT and GATT server/client functionality used by the firmware.
- All existing BLE characteristics and services.
- Notifications and indications.
- Read/write operations and MTU negotiation.
- Pairing, bonding, SMP security, and attribute permissions.
- Application controls and telemetry.
- Audio settings and EQ configuration.
- Firmware uploads and OTA functionality.
- BLE reconnection and lifecycle transitions.
- Existing security and persistent-storage behavior.

BLE must remain operational even while Bluetooth Classic audio is running at its highest supported configuration.

No BLE procedure may stall the audio decoder, transport, or I2S renderer.

## 4. Mandatory Asynchronous and Non-Blocking Architecture

This is one of the highest-priority requirements.

**Every runtime-critical subsystem must be non-blocking, bounded, and event-driven.**

Strictly prohibit:

- Busy-wait loops.
- Spin-waiting for Bluetooth packets or controller credits.
- Waiting in callbacks for another subsystem to finish.
- Unbounded loops processing incoming packets.
- Long synchronous transactions inside Bluetooth callbacks.
- Unbounded lock acquisition.
- Blocking audio output.
- Synchronous file or flash operations on real-time paths.
- Repeated allocation retries.
- CPU-intensive operations inside interrupt handlers.
- Any operation capable of holding the Bluetooth transport hostage.

Use asynchronous state machines, task notifications, timer-driven events, and bounded work queues.

FreeRTOS tasks may sleep or block waiting for events when they have no work. That is acceptable and preferred over CPU-spinning. However, real-time callbacks and externally exposed asynchronous operations must never wait for another subsystem to make progress.

Every scheduler iteration must perform a bounded amount of work and then return control to the scheduler.

Implement explicit backpressure and resource-exhaustion handling. Do not silently lose mandatory protocol replies, security events, acknowledgments, or state transitions.

A full queue must never produce deadlocks, infinite retry loops, invalid memory access, or uncontrolled latency.

Every operation must have documented ownership, completion semantics, and failure handling.

## 5. Extremely Lightweight RAM Architecture

After Bluetooth initialization, device-name configuration, service registration, and connection setup, minimize persistent RAM usage as aggressively as possible.

Create a memory architecture specifically for ESP32 internal SRAM.

Requirements:

- Eliminate unnecessary task stacks.
- Eliminate duplicated packet buffers.
- Eliminate unnecessary packet copies.
- Avoid oversized static arrays.
- Use compact structures and appropriate field widths.
- Use fixed-capacity pools for latency-sensitive resources.
- Use bounded queues with explicit capacities.
- Prefer reusable buffers and transfer of ownership.
- Avoid heap allocations in real-time audio and Bluetooth hot paths.
- Release temporary initialization resources when safe.
- Use DMA-capable memory only where genuinely required.
- Avoid expensive or unnecessary synchronization primitives.
- Minimize task count without creating long-running combined tasks.
- Avoid memory fragmentation during repeated connection cycles.
- Provide complete allocation and deallocation symmetry.

Avoid both excessive static preallocation and unpredictable runtime allocation.

Measure actual heap usage, minimum free heap, largest free block, stack high-water marks, and allocation behavior before and after each major change.

**The finished implementation must use significantly less RAM than the existing host implementation while maintaining feature parity.**

If a proposed memory optimization threatens correctness, protocol obligations, or audio continuity, reject it.

## 6. LHDC V5 — Highest-Priority Audio Optimization

Support LHDC V5 at its highest target configuration:

- Sample rate: 192,000 Hz.
- Sample depth: 24-bit.
- Channels: Stereo.
- Encoded bitrate: 1,000 kbps.
- Original audio fidelity must be preserved.

Use the actual negotiated and supported LHDC V5 configuration. Do not falsely report support for a configuration that the decoder or peer cannot operate.

Optimize the complete decoder pipeline for the ESP32's Xtensa LX6 architecture.

Objectives:

1. Minimize decoder CPU cycles per frame.
2. Minimize intermediate PCM copying.
3. Minimize memory bandwidth consumption.
4. Reduce function-call and dispatch overhead where measurable.
5. Optimize buffer alignment and data access.
6. Minimize unnecessary format conversions.
7. Optimize integer arithmetic and fixed-point operations.
8. Explore suitable Xtensa DSP and multiply-accumulate instructions.
9. Avoid cache-unfriendly accesses and unnecessary branching.
10. Preserve bit-depth precision and audio correctness.
11. Use efficient packet-to-decoder and decoder-to-DMA handoff.
12. Eliminate unnecessary work on the audio-critical path.

Investigate the actual LHDC decoder implementation and determine which operations are consuming the most CPU time.

If the decoder is a closed-source library, optimize its integration and surrounding pipeline without claiming that unavailable decoder internals have been modified.

Use cycle-accurate measurements and profiling rather than assumptions.

### Mandatory Performance Guarantee

At 192 kHz / 24-bit / 1,000 kbps, the audio pipeline must consistently process incoming audio faster than real-time playback requires, with sufficient measured headroom for BLE, Bluetooth control traffic, and other application tasks.

The decoder must not monopolize either CPU.

It must not prevent BLE notifications, GATT writes, AVRCP events, controls, LEDs, configuration updates, or other firmware functions from making progress.

Do not secretly lower the bitrate, sample rate, bit depth, codec quality, or audio precision to meet performance targets.

If the configuration cannot meet its real-time deadline, identify the exact bottleneck with measurements and report the limitation rather than hiding it.

## 7. Preserve Every Existing Audio Codec

Do not optimize the firmware exclusively for LHDC.

Preserve all currently implemented and configured codecs, including where applicable:

- SBC.
- AAC.
- aptX.
- aptX HD.
- aptX Low Latency.
- LDAC.
- LHDC V5.
- Opus.
- Any other existing codec or variant.

Preserve codec negotiation, configuration, transport framing, decoding, sample-rate handling, and original-quality playback.

Maintain correct behavior when switching between codecs or reconnecting to different source devices.

Never disable another codec as a shortcut to reduce RAM or CPU usage.

Any performance optimization must preserve existing codec capabilities and pass regression testing.

## 8. Optimize the Complete Audio Pipeline

Audit and optimize the entire audio processing path:

HCI/VHCI → ACL → L2CAP → AVDTP/RTP → Codec Decoder → DSP/EQ → PCM Buffer → I2S DMA → DAC.

The architecture should minimize copying, waiting, timing jitter, and unnecessary buffering.

Requirements:

- Consistent and correct packet sequence handling.
- Accurate media timestamps.
- Correct RTP payload parsing.
- Proper codec frame assembly.
- Bounded jitter buffering.
- Explicit buffer ownership and lifetime.
- Non-blocking decoder scheduling.
- Reliable DMA refill scheduling.
- Correct sample-rate and channel handling.
- Stable audio clock and buffer occupancy.
- Safe recovery from genuine radio packet loss.
- No software-generated discontinuities.
- No avoidable artificial silence or sample duplication.

Distinguish missing radio packets from host scheduling delays, decoder starvation, buffer underruns, and DMA starvation.

Instrument each stage separately so the exact origin of any audio gap can be identified.

Do not mask transport errors by simply enlarging buffers indefinitely.

## 9. Eliminate EQ Popping Using Integer-Only DSP

Fix the audible popping and clicking whenever EQ parameters are changed during playback.

**All real-time EQ processing must continue using integer or fixed-point arithmetic, not floating-point DSP.**

Implement click-free EQ changes through:

- Smooth parameter transitions.
- Fixed-point coefficient ramps where mathematically safe.
- Safe filter-state handling.
- Crossfading between filter outputs when appropriate.
- Atomic publication of new EQ settings.
- Synchronized updates at safe audio processing boundaries.
- Saturating arithmetic.
- Overflow prevention.
- Consistent gain and headroom management.

Use suitable fixed-point Q formats and sufficient accumulator precision.

Do not introduce unstable intermediate biquad coefficients.

Avoid abruptly resetting filter delay states in a way that creates waveform discontinuities.

The solution must support rapid consecutive EQ changes, EQ enable/disable, gain adjustments, and simultaneous BLE control operations without producing clicks or introducing playback interruptions.

Optimize the DSP for Xtensa LX6 while preserving sound quality.

Test difficult signals, high amplitudes, rapid EQ adjustments, and different audio sample rates.

## 10. Dual-Core Scheduling and Real-Time Priorities

Design the scheduling system for both ESP32 CPU cores.

Keep Bluetooth controller requirements and hardware coexistence behavior in mind.

Separate latency-sensitive responsibilities so that:

- HCI event handling remains responsive.
- Protocol state machines receive bounded execution time.
- Codec decoding has sufficient compute time.
- Audio DMA is serviced before deadlines.
- BLE operations make consistent progress.
- Control and UI operations remain responsive.
- Background storage and configuration operations cannot disrupt audio.

Measure CPU utilization, interrupt duration, execution latency, scheduler delays, and worst-case work duration.

Preserve the current working CPU-affinity and priority decisions unless measured evidence supports a better arrangement.

Do not introduce priority inversion or starvation.

Ensure no task can monopolize execution merely because its queue remains nonempty.

## 11. Memory Safety, Security, and Protocol Correctness

The new stack must behave as a properly engineered Bluetooth implementation, not a collection of special-case packet handlers.

Requirements:

- Correct state machines for every supported protocol.
- Proper handling of malformed packets.
- Strict input length and boundary checks.
- Correct pairing and authentication.
- Secure bond and key handling.
- Correct packet and connection ownership.
- Race-free disconnect and teardown.
- Safe initialization and reinitialization.
- No use-after-free or double-free.
- No stale packet references.
- No incorrect credit reuse.
- No undefined protocol behavior.
- No unbounded protocol queues.
- No unsupported assumptions about controller behavior.

Preserve Bluetooth security requirements and existing access controls.

Never bypass packet integrity checks, authentication requirements, or controller ownership rules to improve throughput.

Do not use undocumented hardware registers or controller behavior without independently verified evidence and explicit justification.

## 12. Full Feature Preservation

Before replacing or optimizing any subsystem, inspect the actual repository and create a comprehensive feature inventory for the A2DP/BLE speaker firmware.

This must include Bluetooth protocols, codecs, security features, audio processing, device controls, storage, OTA, recovery, and all other integrated application functionality.

For every component:

1. Identify its current implementation.
2. Identify the interfaces and resources it depends upon.
3. Document its existing behavior.
4. Implement an equivalent or better replacement where necessary.
5. Test feature parity before retiring the original implementation.

Do not remove functionality just because it is complicated or consumes resources.

If an implementation cannot yet provide an existing feature, preserve the original working path until the replacement is ready.

Do not claim that a protocol or codec is implemented merely because it compiles or connects successfully.

A feature is complete only when its functional, security, lifecycle, and stress tests pass.

## 13. Testing and Performance Validation

Create reproducible benchmarks and automated tests wherever possible.

Mandatory test scenarios include:

- Normal SBC A2DP playback.
- Every other existing codec.
- LHDC V5 at 192 kHz / 24-bit / 1,000 kbps.
- A2DP and BLE running simultaneously.
- Continuous GATT reads, writes, and notifications during audio playback.
- BLE connection and disconnection while audio is active.
- Repeated pairing and reconnection.
- AVRCP commands during playback.
- EQ changes during audio.
- Codec switching.
- Device-name and settings restoration.
- Long-duration streaming.
- Audio playback with OTA/upload activity where supported.
- Rapid connection and disconnection cycles.
- Low-memory conditions.
- Bounded queue saturation and controller backpressure.
- Radio packet loss and recovery.

Record:

- Internal SRAM usage.
- Minimum free heap and largest free block.
- Task stack high-water marks.
- CPU utilization by subsystem.
- Decoder cycles per frame.
- Decoder processing time versus real-time budget.
- HCI packet arrival and delivery timing.
- ACL credit accounting.
- Queue occupancy and high-water marks.
- RTP packet loss and sequence discontinuities.
- Decoder input starvation.
- PCM underruns.
- I2S DMA underruns.
- Worst-case and percentile scheduling latency.
- Bluetooth control and BLE response latency.
- Memory leaks and buffer ownership violations.

Compare the results against the original working implementation under equivalent workloads.

Under a verified loss-free input and a sustainable source rate, the pipeline should demonstrate zero software-induced audio gaps or underruns.

Real RF losses must be reported separately and handled correctly rather than hidden.

## 14. Implementation Strategy

Do not attempt an uncontrolled, all-at-once rewrite.

Proceed through validated engineering stages.

**Phase 1 — Repository Audit**

Inspect every relevant C/C++ source file, build configuration, protocol layer, codec implementation, memory allocation, task, callback, and DMA integration.

Identify existing features, bottlenecks, limitations, and performance baselines.

**Phase 2 — Architecture**

Design the custom HCI/VHCI transport, protocol state machines, memory pools, queue ownership rules, event scheduling, and core allocation.

Produce a clear architecture and RAM budget before implementing major modules.

**Phase 3 — Custom Bluetooth Host**

Implement the custom host incrementally with reproducible tests for HCI, Bluetooth Classic, A2DP, AVRCP, and BLE.

Retain fallback paths until functional parity is demonstrated.

**Phase 4 — Audio and LHDC Optimization**

Profile and optimize decoder execution, packet handling, memory copies, DSP, PCM buffering, and I2S delivery.

Prioritize the maximum LHDC V5 configuration.

**Phase 5 — EQ and DSP**

Implement click-free, integer-only EQ transitions and validate the DSP at every supported audio configuration.

**Phase 6 — Integration and Stress Testing**

Prove simultaneous A2DP, BLE, audio processing, controls, storage, and other existing application features operate correctly for sustained workloads.

**Phase 7 — Final Optimization**

Remove demonstrably redundant allocations, task stacks, data copies, unnecessary abstractions, and scheduling overhead.

Optimize memory and speed only after correctness and feature parity are established.

## 15. Strict Engineering Rules for Codex

Follow these rules throughout the entire project:

1. Read and understand the existing source before rewriting it.
2. Do not invent hardware capabilities or undocumented controller APIs.
3. Do not remove existing A2DP, BLE, codec, DSP, or application features to make performance numbers look better.
4. Do not degrade audio fidelity or reduce codec configurations.
5. Do not substitute polling or busy-waiting for correct asynchronous scheduling.
6. Do not introduce unbounded memory usage or unbounded queues.
7. Do not silently discard mandatory protocol work.
8. Do not bypass Bluetooth security or validation.
9. Do not claim a performance improvement without measurements.
10. Do not assume a successful build means the firmware is correct.
11. Do not claim hardware tests passed unless they were actually performed.
12. Preserve all existing in-scope firmware functionality and external interfaces.
13. Every major modification must have clear reasoning and verification.
14. Favor the smallest correct implementation over unnecessary abstraction.
15. Optimize for real ESP32 hardware behavior, not desktop assumptions.

When a hardware, radio, or proprietary codec limitation prevents a goal from being achieved, provide a precise explanation supported by measurements and propose the smallest defensible next test or change.

## 16. Final Definition of Success

The project is complete only when the custom host stack:

- Independently handles the required Bluetooth Classic and BLE protocols through HCI/VHCI.
- Is fully asynchronous and free of busy-waiting and unbounded runtime execution.
- Uses less internal SRAM than the original implementation.
- Has deterministic and bounded memory ownership.
- Preserves every currently available in-scope firmware feature.
- Supports every existing audio codec.
- Reliably handles the highest supported LHDC V5 configuration with measured real-time processing headroom.
- Keeps BLE, AVRCP, controls, and other firmware functionality responsive during maximum audio load.
- Produces continuous audio without software-induced stutters.
- Eliminates audible EQ-change popping using integer-only DSP.
- Maintains pairing, security, reconnection, and protocol correctness.
- Passes reproducible functional, performance, and long-duration stress tests.

**Ultimate objective: Deliver a truly original, production-quality, exceptionally fast and memory-efficient Bluetooth Classic A2DP + BLE host stack engineered for the original ESP32, achieving maximum audio fidelity and real-time performance without compromising any in-scope functionality.**

Begin by auditing the existing codebase and establishing measured baselines. Then implement, integrate, and validate the new architecture in controlled increments. Do not stop at a design document; produce working, tested source code and a clear record of what has actually been verified.