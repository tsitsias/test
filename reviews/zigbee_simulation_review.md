# Polykarpos Simulation Review

## Context
The Zigbee simulation orchestrated in `Polykarpos-Simulation` models coordinator, router, and end-device nodes under multiple traffic patterns with energy tracing. The goal is to generate realistic, pseudo-random traffic for attack detection datasets while maintaining accurate energy consumption measurements.

## Key Observations & Risks

1. **Shared Random Variable Instances**  
   Several traffic models reuse a single random variable instance (`expRv`, `normalRv`, `rv`) across different nodes or flows. Because ns-3's RNG streams are deterministic per object, reusing the same stream couples inter-arrival times between unrelated traffic pairs, reducing entropy between runs and producing correlated bursts.

2. **Packet Payload Handling**  
   `SendData` manually allocates a raw buffer with `new` and never deletes it, leaking memory on every packet. While ns-3 frees packet data eventually, repeated leaks during long simulations distort energy models tied to CPU activity and can crash extended runs.

3. **Static NSDU Handles & Route Discovery Flags**  
   `req.m_nsduHandle` is hardcoded to `32` while a separate handle counter is stored in `g_handleInfo`. Because the request uses the constant value, confirms cannot be correlated reliably with `g_handleInfo`, defeating statistics tracking. `ENABLE_ROUTE_DISCOVERY` is applied indiscriminately, creating repeated route discoveries even over stable links and inflating energy cost.

4. **Traffic Model Initialization**  
   - `SetupPoissonTraffic`, `SetupAdaptiveCBR`, and `SetupLoadBasedTraffic` schedule transmissions immediately for nodes that may not have joined yet, generating `Null` stack dereferences or dropped packets.  
   - Event-driven traffic installs a join callback but still calls `SetNlmeJoinConfirmCallback` later, overwriting the event handler.

5. **Energy Model Instrumentation**  
   Energy state transitions assume each PHY trace fires in pairs (begin/end). Drops or asynchronous events can leave the device in TX/RX state and never return to idle. Moreover, `PacketIdleTraceRx` ignores its parameters and duplicates `PacketIdleTrace`.

6. **Logging and Metrics**  
   The CSV log for remaining energy is opened globally without checking the output path (or creating directories) and is never flushed/closed on early exits. Packet statistics are only printed at the end; intermittent checkpoints would help correlate traffic bursts with energy dips.

7. **Randomness Seeding**  
   `RngSeedManager::SetSeed(time(NULL))` runs after scheduling events, and `SetRun(3)` is constant. Without per-run stream assignment, different traffic models may share identical stream IDs, limiting pseudo-random variation.

8. **Scalability & Maintainability**  
   The file mixes network setup, callbacks, traffic logic, and statistics in a single translation unit (~1k LOC). This complicates testing, code reuse, and future attack modeling.

## Recommended Improvements

### Randomness Isolation & Reproducibility
- Instantiate per-flow/per-node random variables with `CreateObject<...>()` and assign unique stream indices via `SetStream`.  
- Move seed/run configuration before any RNG use and expose CLI options for reproducible experiments.

### Memory Safety & Packet Construction
- Replace manual buffer allocation with `Create<Packet>(payloadSize)` using `ns3::Packet`'s zero-filled constructor or `MakeUnique<uint8_t[]>`. Avoid leaks by using smart pointers or stack storage.

### Correct Handle Tracking
- Assign `req.m_nsduHandle = handle;` so confirm callbacks map to the stored metadata.  
- Optionally disable route discovery (`DISABLE_ROUTE_DISCOVERY`) for known neighbors and use `ZigbeeNwk::NlmeRouteDiscoveryRequest` explicitly when needed.

### Traffic Model Robustness
- Delay traffic scheduling until after join confirmation. Maintain a per-node `Simulator::Schedule` triggered from `NlmeJoinConfirm` to avoid sending before association.  
- Split event-driven callbacks so the default join confirm forwards to `OnNodeJoin`.  
- For `OriginalTrafficCreator`, cap the pair matrix or stagger transmissions using per-flow jitter to avoid O(N^2) bursts.

### Energy Instrumentation
- Implement explicit state machines for TX, RX, idle, and sleep currents with timers to avoid stuck states.  
- Use `SimpleDeviceEnergyModel::SetCurrentA` within guard functions that verify current state transitions.  
- Flush `g_energyCsv` periodically (e.g., every `EnergyRemainingTrace` call) and ensure the output directory exists.

### Logging & Monitoring
- Extend packet logging to include MAC retries, route discovery status, and energy snapshots.  
- Serialize statistics after each traffic model iteration, including metadata (traffic type, seed, node count) in the CSV header.

### Modularity & Testing
- Break the program into components: configuration parser, topology builder, traffic scheduler, energy logger, and metrics collector.  
- Provide unit-style simulations (`TestSuite`) for each traffic model to validate scheduling order, join handling, and energy accounting.

### Attack Scenario Hooks
- Introduce interfaces for attack behaviors (e.g., selective forwarding, flooding) that can reuse the same traffic scaffolding.  
- Record per-node per-state energy to serve as features for the neural network dataset.

Implementing these changes will stabilize energy measurements, increase pseudo-random diversity, and deliver trustworthy datasets for attack detection research.
