# Distributed Traffic Simulation — Public Demo

This repository contains two deliberately separated pieces:

1. **`demo/`** — a small, independently written MPI shared-memory demonstration. It is a runnable teaching/demo program and does not depend on QarSUMO or SUMO internals.
2. **`sumo-modifications/candidate-files/`** — selected files from the modified SUMO source snapshot supplied for this work, chosen because they contain shared-state integration markers. These are review candidates, **not a verified, buildable patch** against an identified upstream commit.

## What the demo demonstrates

- Splitting MPI ranks into a shared-memory communicator.
- Allocating a shared vehicle-state array with `MPI_Win_allocate_shared`.
- Querying the shared segment from another rank.
- Publishing and reading vehicle ID, position, speed, and lane identifiers.
- Synchronizing visibility of updates with MPI window synchronization and barriers.

The demo is intentionally small. It is **not** a full traffic simulator, does not implement SUMO vehicle dynamics, and does not establish correctness of the experimental research implementation.

## Build and run the standalone demo

Requirements: an MPI implementation (e.g. Open MPI or MPICH), a C++ compiler, and `make`.

```bash
cd demo
make
mpirun -np 2 ./shared_vehicle_state_demo
```

Run the two ranks on the same host, because the example specifically demonstrates MPI shared-memory windows. Some clusters require `mpirun --oversubscribe -np 2 ...` for local testing.

Expected output includes a rank reading the vehicle state written by the shared-memory leader. The exact output ordering can vary.

## About the SUMO source candidates

The supplied archive `src(1).zip` did not identify an exact upstream Git commit in the archive itself. The full uploaded `sumo.zip` and older `sumo-org.zip` also appear to be different snapshots, so this repository does not assume that either is the correct baseline for a source diff.

Before publishing `sumo-modifications/candidate-files/`:

1. Identify the exact SUMO upstream revision used for this modified source tree.
2. Compare each candidate file against that exact revision and generate a reviewed patch.
3. Preserve existing copyright and license notices; include the applicable SUMO license text.
4. Remove any QarSUMO-only or project-confidential material and verify that you have permission to release the changes.
5. Build and test against the matching SUMO revision before describing it as working code.

The files are separated from the runnable demo so you can review them before committing. Do not represent the candidate file set as a complete SUMO source release or a verified patch.

## Repository scope

No QarSUMO source archive, QarSUMO application source, datasets, simulation outputs, or private project files are included. The standalone demo is independently written for this portfolio package.
