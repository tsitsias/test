# Polykarpos Zigbee Simulation

This repository now contains an updated ns-3 Zigbee simulation (`src/polykarpos_simulation.cc`) that instrument energy usage, packet statistics, and traffic behaviour for coordinator, router, and end devices. The implementation focuses on reproducible pseudo-random traffic suitable for training anomaly detection models.

## Highlights
- Modular configuration (`SimulationConfig`) with CLI flags for topology, runtime, payload size, RNG seed/run, and traffic model selection.
- Traffic manager that delays scheduling until nodes successfully join and isolates random streams per flow.
- Memory-safe packet construction and accurate NSDU handle tracking for confirm callbacks.
- Energy controllers that drive `SimpleDeviceEnergyModel` currents based on PHY traces, writing logs to `energy_ns3_log.csv` in the chosen output directory.
- Final packet statistics exported to `packet_statistics.csv` alongside console summaries, with optional time-series samples in `traffic_timeseries.csv`.

## Building & Running
1. Ensure you have ns-3.45 (or newer) built with Zigbee support.
2. Copy `src/polykarpos_simulation.cc` into your ns-3 scratch directory or integrate it into a custom module.
3. Build with waf, for example:
   ```bash
   ./waf build --run "scratch/polykarpos_simulation --router_number=3 --device_number=6 --payload_size=72 --traffic_model=poisson --seed=12345 --run=7 --stats_interval=60 --output_path=results"
   ```
4. Inspect the generated CSV files in the provided `--output_path` for energy traces and packet statistics.

## Traffic Models
- `poisson`: Independent exponential inter-arrivals tailored per role.
- `adaptive`: CBR-style intervals that expand with node activity to avoid overload.
- `event`: Join-triggered flows with randomized start delays, ideal for event-driven datasets.
- `loadbased`: Gaussian interval selection with dynamic destinations based on role.
- `original`: Complete pairwise exchanges with configurable jitter for regression studies.

Each flow assigns a dedicated RNG stream, ensuring pseudo-random reproducibility across runs while avoiding inter-flow correlation.

## Outputs
- `energy_ns3_log.csv`: Remaining energy by node over time.
- `packet_statistics.csv`: Aggregate packet totals per node written after simulation completes.
- `traffic_timeseries.csv` (optional): Aggregate sent/received/dropped packet counts sampled every `--stats_interval` seconds.
- `simulation_metadata.json`: Snapshot of the configuration used for the run (topology, RNG settings, timing parameters).
