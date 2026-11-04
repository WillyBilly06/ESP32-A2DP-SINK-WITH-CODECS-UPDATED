# Independent BLE product control

The probe retains the diagnostic service and adds the baseline product service
`12345678-1234-1234-1234-123456789000`, with command, status and meter UUIDs
ending in `9001`, `9002` and `9003`. Command writes are bounded and copied on
the host owner; parsing, NVS and provider operations run on lazily created
workers. Product status and meter notifications have independent CCCDs. Status
notifications require an enabled status CCCD or a genuinely restored bonded
subscription; commands do not modify the CCCD. Responses wait in the bounded
job pool while status notifications are disabled and are discarded on disconnect.
Queue exhaustion rejects further commands; status subscriptions emit a chunked
full-status response. The old baseline firmware enables status notifications on
the first command if it did not observe a CCCD write, apparently to accommodate
clients whose subscription write it missed. This implementation does not infer
subscription from a command. No product client source was found locally, so its
actual subscription sequence remains unverified.

`probe_gatt_control_configure()` must run before `probe_gatt_init()`. Prepare a
real initial state before advertising, on a startup worker, and publish the
loaded EQ and flags before enabling the service. `dbt_ble_nvs_load_state()` is
read-only and expects caller-provided defaults for missing data, including
firmware version and sound-presence bits. It reads existing `audio` EQ, control,
name, LED effect and sound-mute keys and existing `led` brightness/settings;
the settings blob also supplies colors, speed and gradient. It leaves the input
unchanged on read errors and makes no migrations or writes. Sound-presence bits
still require a real sound provider. The advertised GAP probe name is currently
fixed, even if the product status reports a saved name.

Each EQ or preset job that starts invokes `dbt_ble_nvs_commit_eq()` once, including
identical changes and work accepted immediately before disconnect. Allocation or
worker-start failure is reported as failure, never a successful EQ change. The function
returns success only after `nvs_commit`. Publication uses a field-only,
nonblocking hook, preserving independently controlled volume and flags. A
failed publication after commit produces an error response, while the durable
value stays current. The worker makes two bounded publication-only retries;
`probe_gatt_retry_eq_publication()` can retry from the owner without another
commit. It returns `DBT_BUSY` if any EQ job is still running. Flags use the
corresponding `probe_gatt_retry_control_publication()` contract. The owner
must schedule retries, and providers must allow worker/owner calls safely.
There is no success notification for a later recovery after an error response.

The generic command provider receives a handle and generation and runs on its
own worker. Its reply must describe a completed operation, not admission into
another queue. Name, LED, sound and OTA are unavailable until a real provider
is wired; missing providers return errors. Already-running provider operations
cannot be canceled by this adapter, so providers must check the peer generation
when an operation requires a still-connected peer. Queued unstarted non-EQ
jobs are canceled on disconnect. No result is delivered to a reused generation.
External state changes and provider-generated asynchronous status events need
additional owner-side integration; the status snapshot otherwise changes only
through completed commands. The meter API is owner-only and best-effort.

Two workers use 4096-byte stacks and terminate after disconnect and quiescence;
queues and at most four bounded jobs remain allocated while the service is
active. Three generic jobs are permitted so one slot remains for EQ. Queue or
task allocation failure produces an error rather than a success; it does not
offer persistence for work that could not start. Actual flash stall, scheduling,
stack and heap behavior on target remain unmeasured.

Bonded CCCD persistence has no provider: if the ATT connection is marked
bonded, CCCD writes are rejected rather than falsely claiming durable storage.
Unbonded CCCD behavior and the diagnostic service remain available. Security
policy for mutating commands is an explicit nonblocking authorization hook;
the application must choose it and preserve diagnostic access intentionally.

Local checks: `python native_bt/tools/test_ble_control.py` exercises the real
ATT server and parser with deterministic task, queue and NVS shims;
`python native_bt/tools/test_probe_gatt_compile.py` compiles the three product
units with ESP32 compiler flags. Neither is evidence of target linking, real NVS
commit timing, radio operation, phone interoperability or audio quality.
