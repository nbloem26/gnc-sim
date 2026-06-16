# Distributed simulation — HLA / DIS / DDS federation (issue #47)

gnc-sim can put a run on a distributed-simulation federation: publish each step's entity + track
state to other simulators, and **record + deterministically replay** a federated run. This lives in
its own opt-in module (`core/include/gncsim/interop/`, `core/src/interop/`) and is **never touched by
the pure GNC hot path** — `runSimulation()` does not reference any of it, so the default
native/WASM trajectory and golden runs are byte-identical with the feature off (AGENTS.md golden
rules #1/#2).

> **What ships working vs. documented.** **DIS is the concrete, tested leg** — a self-contained
> IEEE-1278 wire protocol implemented with std-only code (no external library). **HLA (IEEE 1516)
> and DDS (OMG)** require an external RTI / DDS vendor, which we deliberately do **not** vendor; for
> those we ship the **adapter interface** (the exact seam DIS implements) plus the integration recipe
> below. No new third-party dependencies are added.

## Architecture

```
                       ┌────────────────────────────────────────────┐
   SimResult ─────────▶│  snapshotsFromResult()  → StepSnapshot[]    │
   (telemetry frames)  └───────────────────┬────────────────────────┘
                                           │  (per-step entity + track state, ENU, SI units)
                                           ▼
                                   ┌──────────────┐   attach order = fixed, deterministic fan-out
                                   │  MessageBus  │
                                   └──────┬───────┘
                          ┌───────────────┼────────────────────────────┐
                          ▼               ▼                            ▼
                   ┌────────────┐  ┌──────────────┐            ┌────────────────┐
                   │ DisAdapter │  │ FileRecorder │            │ HLA / DDS adapter
                   │ (IEEE 1278)│  │ (record/replay)           │ (interface + docs)
                   └─────┬──────┘  └──────────────┘            └────────────────┘
                         ▼
                   ┌────────────┐  Loopback (hermetic, in-process — used by CI tests)
                   │ ITransport │  UDP       (real IEEE-1278 wire, opt-in, behind a runtime flag)
                   └────────────┘
```

* **`StepSnapshot`** (`interop/Federation.hpp`) — one step's published state: a list of
  `EntityStateMsg` (truth kinematics of each entity in the ENU world frame, SI units) and
  `TrackMsg` (fused-track estimates, issue #5). `snapshotsFromResult()` drains a finished
  `SimResult` into this sequence (interceptor = entity id 1, threat = id 2).
* **`MessageBus`** (`interop/MessageBus.hpp`) — a thin, ordered multiplexer. A driver attaches
  adapters, then calls `start()` / `publish()` per step / `stop()`. Adapters are visited in **attach
  order** on every call, so a fixed adapter set produces deterministic side effects.
* **`IFederateAdapter`** — the publish/subscribe seam: `onStart`, `publish(snapshot)`, `onStop`.
  **This is the one interface every federation leg implements.** DIS, the file recorder, and any
  future HLA/DDS leg are all just `IFederateAdapter`s.

## DIS leg (working, tested)

`interop/Dis.hpp` is a std-only IEEE-1278.1 codec (big-endian / network byte order via
`interop/ByteIo.hpp`):

* **PDU header** (12 bytes): protocol version (5), exercise id, PDU type, family, timestamp, length.
* **Entity State PDU** (type 1): entity id triplet, force id, entity kind, sim time, ENU position,
  linear velocity, Euler orientation, angular velocity. Position / velocity / orientation use the
  protocol's **32-bit-float** field resolution, so an encode→decode round-trip is exact to `float`
  precision (asserted within that resolution by `tests/distributed_test.cpp`).
* **Track/Detection PDU** (experimental type 129): the fused-track message.

`DisAdapter` (`interop/DisAdapter.hpp`) encodes each snapshot's entities/tracks to PDUs and sends
them over an injected `ITransport`. `drainDis()` is the receive side — it pulls datagrams, dispatches
on PDU type, and decodes back into typed messages.

### Transports

* **`LoopbackTransport`** — an in-process FIFO. Two endpoints can be cross-wired (`connectTo`) so one
  federate's sends land in the other's inbox, with **no socket and no port**. This is what the CI
  test uses (two endpoints exchanging entity state, zero flake risk).
* **`UdpTransport`** — real broadcast/unicast UDP on the DIS exercise port (3000), POSIX sockets,
  std-only. Compiled **only** when the build defines `GNCSIM_INTEROP_UDP` and used only behind an
  explicit runtime choice, so the default build pulls in no socket headers and binds no port. Never
  exercised by the deterministic test.

## Record + deterministic replay (working, tested)

`FileRecorder` (`interop/Recording.hpp`) is an `IFederateAdapter` that serialises every snapshot to a
fixed, endian-explicit byte stream (`GNCR` magic + version, then full **f64 bit-pattern** fields — no
host layout, no float rounding). Therefore:

* the same run records to **byte-identical** bytes on any host;
* `replay()` reconstructs the exact `StepSnapshot` sequence;
* **record → replay → re-record reproduces the same bytes** (the determinism invariant the test
  asserts).

A replayed snapshot stream can be re-published through a fresh `MessageBus` (e.g. with a
`DisAdapter` attached) to drive a federation from a canned run — replaying a federated exercise
deterministically.

### CLI

```bash
gncsim --config configs/homing_3dof.json --out runs/run_001 \
       --federation-record runs/run_001/federation.gncr
```

Opt-in: with no `--federation-record` flag the CLI behaves exactly as before. Two recordings of the
same config are byte-identical.

## Plugging in HLA (IEEE 1516) and DDS (OMG)

Both need an external runtime we intentionally do not vendor. Each is just **one more
`IFederateAdapter`** plus a vendor dependency wired in at *your* build, kept out of the default
`core` build:

### HLA adapter sketch

```cpp
class HlaAdapter : public gncsim::interop::IFederateAdapter {
 public:
  // ctor: connect to the RTI (RTIambassador), join the federation execution, publish the object
  //       classes / interaction classes from your FOM (e.g. BaseEntity.PhysicalEntity,
  //       and a Track interaction).
  void onStart(uint16_t ex, uint16_t site, uint16_t app) override { /* registerObjectInstance per entity */ }
  void publish(const StepSnapshot& s) override {
    // For each entity: updateAttributeValues(handle, {Position, Velocity, Orientation, ...}).
    // For each track:  sendInteraction(trackClass, params).
    // Map ENU (Types.hpp) → the FOM's geocentric SpatialFP at this boundary.
  }
  void onStop() override { /* resignFederationExecution */ }
};
```

Link your build against an RTI (e.g. Pitch pRTI, MAK, or the open-source Portico) and its generated
FOM stubs. The mapping work is: gnc-sim ENU world frame → the FOM's geocentric/topocentric spatial
representation, applied **inside the adapter** (the bus stays frame-agnostic, SI, ENU).

### DDS adapter sketch

```cpp
class DdsAdapter : public gncsim::interop::IFederateAdapter {
 public:
  // ctor: create a DomainParticipant, define IDL topics (EntityState, Track), create DataWriters.
  void publish(const StepSnapshot& s) override {
    // writer_entity_.write(toIdl(entity));  // one sample per entity
    // writer_track_.write(toIdl(track));    // one sample per track
  }
};
```

Link against a DDS vendor (RTI Connext, eProsima Fast DDS, OpenDDS, Cyclone DDS) and compile your
IDL. The `EntityStateMsg` / `TrackMsg` structs map 1:1 to IDL structs.

Because the seam is exactly three methods, the heavy RTI/DDS runtime lives entirely outside this
repo, and the default native/WASM build never sees it.

## Invariants

* **Opt-in & out of the hot path.** `runSimulation()` does not reference `interop/`; the default
  build + CTest are unchanged and `homing_3dof` is byte-identical.
* **No new dependencies.** DIS, the bus, the recorder, and the loopback/UDP transports are std-only.
* **Deterministic.** Record → replay → re-record is byte-identical (big-endian, f64 bit patterns).
* **Hermetic tests.** `tests/distributed_test.cpp` uses the loopback transport — no real port is
  bound, so CI cannot flake. Real UDP is behind `GNCSIM_INTEROP_UDP`.
