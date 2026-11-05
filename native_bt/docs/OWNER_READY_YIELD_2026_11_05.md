# Cooperative owner yield with equal-priority decoder

The original [priority6 RF results](DECODER_PRIORITY_RF_RESULTS_2026_11_05.json)
reach 421696 source frames at DMA but still have 577856 underrun frames, 540
queue-full drops, one core1 idle watchdog and a descriptor-order fault after a
10464 us EOF gap. First queue rejection is 628 bytes at sequence17 with eight
queued. Source/ELF/capture receipts are preserved, and original owning inputs
are extracted and hash-checked before this follow-up. Released baseline return
and phone pause are verified. Different source content from priority5 prevents
a controlled percentage-speedup claim.

The decoder's event loop already yields after each bounded runnable batch. The
host instead immediately continues when its combined wait calculation is zero.
For the finite priority6 experiment, the host now also calls public `taskYIELD`
after that bounded iteration. Equal-priority runnable decoder work can receive
service before the host's entire sequence of immediate iterations completes.
No vTaskDelay, polling interval, task, buffer or clock/codec/DSP change is added.
Default decoder priority5 retains its previous owner-loop behavior.

`scheduler_layout.owner_ready_yield` identifies this revision of the experiment;
`owner_wait_reasons.runnable_yields` counts requests, not effective context
switches or decoder progress. Controller priority and all asynchronous ownership
rules remain intact. A yield cannot provide CPU headroom when required work is
itself over budget, and it does not guarantee service for lower-priority idle.
Keep watchdog and descriptor checks enabled.

The build receipt binds the derived mode to the actual main priority definition.
Seven actual owner-yield admission cases, existing priority/finite guards,
real-codec host tests at priorities5/6, the ccache target build, source/image
admission and limited callback placement audit pass. These do not establish
radio scheduling, continuous audio or feature parity. Hardware measurement is
the next gate.
