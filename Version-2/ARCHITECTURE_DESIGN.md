# Backend Architecture Design: Race-Condition-Safe Drone Ground Control System

## Executive Summary

This document defines a **strict producer-consumer architecture** with **single-threaded processing ownership** and **read-only API access** to ensure zero race conditions in a real-time drone ground control system.

---

## 1️⃣ Thread & Process Layout

### Process Structure
- **Single Python process** containing all components
- **Main thread**: Orchestrates startup/shutdown, hosts FastAPI server
- **3 dedicated threads**: TCP Receiver, Processing Loop, FastAPI (Uvicorn worker)

### Thread Responsibilities

#### **Thread 1: TCP Receiver Thread** (`tcp_receiver_thread`)
**Purpose**: Receive and decode TCP packets from Raspberry Pi

**ALLOWED Operations**:
- ✅ Accept TCP connections
- ✅ Receive raw TCP packets (blocking `recv()`)
- ✅ Decode packet header (4-byte size)
- ✅ Decode JSON metadata
- ✅ Decode JPEG image (`cv2.imdecode`)
- ✅ Create `(frame, telemetry)` tuple
- ✅ Push tuple into **bounded queue** (`queue.put_nowait()` or `queue.put()` with timeout)
- ✅ Handle TCP disconnections gracefully
- ✅ Log TCP-level events (connection, disconnection, decode errors)

**EXPLICITLY FORBIDDEN**:
- ❌ **NEVER** run YOLO inference
- ❌ **NEVER** access survivor state
- ❌ **NEVER** write waypoint files
- ❌ **NEVER** call FastAPI endpoints
- ❌ **NEVER** display OpenCV windows
- ❌ **NEVER** mutate shared state (except queue push)
- ❌ **NEVER** block indefinitely (use timeout on queue operations)

**Queue Behavior**:
- If queue is **full**: Drop frame (log warning), continue receiving
- If queue is **empty**: Block waiting for next packet (acceptable)
- Queue size: **Configurable** (default: 10 frames)

---

#### **Thread 2: Processing Loop Thread** (`processing_thread`)
**Purpose**: Single owner of all state mutations and heavy computation

**ALLOWED Operations**:
- ✅ Consume frames from queue (`queue.get()` with timeout)
- ✅ Run YOLO inference (`model.predict()`)
- ✅ Perform frame stabilization (ORB + RANSAC)
- ✅ Compute geolocation (pixel → lat/lon)
- ✅ Update survivor state (with lock acquisition)
- ✅ Write waypoint files (per survivor)
- ✅ Update temporal confirmation state machine
- ✅ Emit WebSocket events (via thread-safe event queue)
- ✅ Handle frame dropping (if queue is empty, skip iteration)

**EXPLICITLY FORBIDDEN**:
- ❌ **NEVER** read from TCP socket directly
- ❌ **NEVER** accept TCP connections
- ❌ **NEVER** serve HTTP requests (FastAPI handles this)
- ❌ **NEVER** display OpenCV windows (`cv2.imshow`)
- ❌ **NEVER** mutate survivor state without lock
- ❌ **NEVER** block indefinitely (use timeouts)

**State Ownership**:
- **Sole writer** of `survivor_registry` (dict of Survivor objects)
- **Sole writer** of waypoint files
- **Sole writer** of detection results

---

#### **Thread 3: FastAPI/Uvicorn Worker Thread** (`fastapi_thread`)
**Purpose**: Serve REST API and WebSocket connections

**Implementation Note**: 
- Uvicorn spawns its own event loop (async/await)
- **No manual threading required** — Uvicorn handles concurrency internally
- Treat FastAPI/Uvicorn as **logically isolated** from other threads
- In implementation: Start Uvicorn normally (`uvicorn.run()`), it manages its own event loop
- Design remains unchanged — this is an implementation awareness point

**ALLOWED Operations**:
- ✅ Accept HTTP connections
- ✅ Serve REST endpoints (read-only survivor queries)
- ✅ Accept WebSocket connections
- ✅ Read survivor state (with lock acquisition, create snapshot)
- ✅ Send WebSocket messages (from event queue)
- ✅ Serve waypoint file downloads (read-only file access)
- ✅ Handle POST `/api/survivors/{id}/dispatch` (read-only state query + write to dispatch queue)

**EXPLICITLY FORBIDDEN**:
- ❌ **NEVER** mutate survivor state directly
- ❌ **NEVER** run YOLO inference
- ❌ **NEVER** read from frame queue
- ❌ **NEVER** write waypoint files
- ❌ **NEVER** block processing thread (all operations must be fast)
- ❌ **NEVER** perform heavy computation

**State Access Pattern**:
- **Read-only** access to survivor state
- Acquire lock → create immutable snapshot → release lock → process snapshot
- Dispatch requests go to a separate queue (processed by processing thread)

---

### Thread Communication Channels

```
┌─────────────────┐
│ TCP Receiver    │──[bounded queue]──>┌─────────────────┐
│ Thread          │                    │ Processing Loop │
└─────────────────┘                    │ Thread          │
                                       └────────┬────────┘
                                                │
                                                │ [lock-protected]
                                                │ [event queue]
                                                ▼
                                       ┌─────────────────┐
                                       │ Survivor State  │
                                       │ (in-memory dict)│
                                       └────────┬────────┘
                                                │
                                                │ [read-only]
                                                ▼
                                       ┌─────────────────┐
                                       │ FastAPI Thread  │
                                       │ (REST + WS)     │
                                       └─────────────────┘
```

---

## 2️⃣ Data Flow (Step-by-Step)

### Phase 1: TCP Reception → Queue

1. **TCP Receiver Thread**:
   - Blocks on `socket.recv(4)` waiting for packet size header
   - Receives full payload (`recv_exact(size)`)
   - Parses JSON metadata (frame_id, timestamp, lat, lon, alt_agl, heading, pitch, roll)
   - Decodes JPEG: `cv2.imdecode(payload[meta_end:], cv2.IMREAD_COLOR)`
   - Creates tuple: `(frame: np.ndarray, telemetry: DroneState)`
   - **Attempts queue push**: `frame_queue.put_nowait((frame, telemetry))`
     - **If queue full**: Log warning, drop frame, continue
     - **If success**: Continue to next packet

**Lock Usage**: None (queue is thread-safe)

---

### Phase 2: Queue → Processing

2. **Processing Loop Thread**:
   - **Blocks on queue**: `frame_queue.get(timeout=0.1)`
     - **If timeout**: Skip iteration, check shutdown flag, continue
     - **If frame received**: Proceed to processing
   - **Frame validation**: Check pitch/roll (reject non-nadir frames)
   - **Frame stabilization**: Compute homography (ORB + RANSAC)
   - **YOLO inference**: `model.predict(stabilized_frame, ...)`
   - **For each detection**:
     - Extract bounding box center (u, v)
     - Project to map coordinates (homography transform)
     - Convert map pixel → lat/lon
     - Validate distance (`haversine` check)
     - **Acquire survivor lock**: `survivor_lock.acquire()`
     - **Update survivor state**: Call `clusterer.add(lat, lon, altitude)`
     - **Check confirmation**: If `frames_seen >= MIN_FRAMES_SEEN` and status is CANDIDATE:
       - Transition: `CANDIDATE → CONFIRMED`
       - Generate waypoint file: `waypoint_writer.save_survivor(lat, lon)`
       - Update survivor record: `survivor.waypoint_file = filename`
       - **Emit event**: Push `SURVIVOR_CONFIRMED` to WebSocket event queue
     - **Release lock**: `survivor_lock.release()`

**Lock Usage**: `survivor_lock` (acquired during state mutation only)

---

### Phase 3: Survivor Confirmation → State Update

3. **Temporal Confirmation Logic** (inside processing thread):
   - **Spatial clustering**: Check if detection is within `radius = max(altitude * 0.7, 5.0)` meters of existing cluster
   - **If match**: Increment `frames_seen`, update centroid (running average)
   - **If new**: Create new cluster with `frames_seen = 1`
   - **Confirmation trigger**: When `frames_seen == MIN_FRAMES_SEEN`:
     - Status: `CANDIDATE → CONFIRMED`
     - Generate waypoint file
     - Emit `SURVIVOR_CONFIRMED` event

**Lock Usage**: `survivor_lock` (entire clustering operation is atomic)

---

### Phase 4: UI Update (WebSocket)

4. **WebSocket Event Emission** (processing thread → FastAPI thread):
   - Processing thread pushes events to `websocket_event_queue`:
     ```python
     event = {
         "type": "SURVIVOR_CONFIRMED",
         "survivor_id": survivor.id,
         "data": {...}
     }
     websocket_event_queue.put_nowait(event)
     ```
   - **FastAPI thread** (background task):
     - Polls `websocket_event_queue.get(timeout=0.1)`
     - Broadcasts to all connected WebSocket clients
     - **No lock needed** (queue is thread-safe, event data is immutable)

**Event Queue Overflow Policy**:
- **If queue is full**: **Drop oldest events** (FIFO removal)
- **Rationale**: UI needs **latest state**, not historical events
- **Implementation**: Use bounded queue with `maxsize`, implement custom `put()` that removes oldest item when full
- **Alternative**: Use `collections.deque` with `maxlen` (automatically drops oldest on append when full)

**Lock Usage**: None (queue is thread-safe, events are immutable)

---

### Phase 5: REST API Read Access

5. **FastAPI REST Endpoint** (FastAPI thread):
   - **GET `/api/survivors`**:
     - Acquire `survivor_lock`
     - Create snapshot: `snapshot = {id: surv.to_dict() for id, surv in survivor_registry.items()}`
     - Release lock
     - Return JSON response (process snapshot without lock)
   - **GET `/api/survivors/{id}`**:
     - Acquire lock → get survivor → create dict copy → release lock
     - Return JSON (process copy without lock)
   - **GET `/api/survivors/{id}/waypoint`**:
     - Acquire lock → read `survivor.waypoint_file` path → release lock
     - Read file from disk (read-only, no lock needed)

**Lock Usage**: `survivor_lock` (acquired for snapshot creation, released before processing)

---

### Phase 6: Dispatch Request (POST)

6. **POST `/api/survivors/{id}/dispatch`** (FastAPI thread):
   - Validate survivor exists (acquire lock → check → release lock)
   - Push dispatch request to `dispatch_queue`: `dispatch_queue.put_nowait(survivor_id)`
   - Return `{"status": "queued"}`
   - **Processing thread** (separate iteration):
     - Polls `dispatch_queue.get(timeout=0.1)`
     - Acquires `survivor_lock`
     - Updates status: `CONFIRMED → DISPATCHED`
     - Emits `SURVIVOR_DISPATCHED` event
     - Releases lock

**Lock Usage**: `survivor_lock` (acquired by processing thread during state mutation)

---

## 3️⃣ Survivor Data Model & State Machine

### Data Model

```python
@dataclass
class Survivor:
    id: str                    # Sequential ID format: "surv_001", "surv_002", "surv_003", ...
    lat: float                 # Confirmed latitude
    lon: float                 # Confirmed longitude
    status: str                # CANDIDATE | CONFIRMED | DISPATCHED
    frames_seen: int           # Temporal confirmation counter
    first_seen: float          # Unix timestamp of first detection
    confirmed_at: float        # Unix timestamp when confirmed (None if CANDIDATE)
    dispatched_at: float       # Unix timestamp when dispatched (None if not DISPATCHED)
    waypoint_file: str         # Path to waypoint file (None if CANDIDATE)
    detection_history: List[Tuple[float, float, float]]  # [(lat, lon, timestamp), ...]
    
    def to_dict(self) -> dict:
        """Create immutable snapshot for API responses"""
        return {
            "id": self.id,
            "lat": self.lat,
            "lon": self.lon,
            "status": self.status,
            "frames_seen": self.frames_seen,
            "first_seen": self.first_seen,
            "confirmed_at": self.confirmed_at,
            "dispatched_at": self.dispatched_at,
            "waypoint_file": self.waypoint_file
        }
```

**Survivor ID Strategy**:
- **Format**: Sequential IDs (`surv_001`, `surv_002`, `surv_003`, ...)
- **Rationale**:
  - ✅ Easier UI display and operator reference
  - ✅ Matches waypoint filename pattern (`survivor_001_*.waypoints`)
  - ✅ Simplifies operator mapping (survivor ID ↔ waypoint file)
  - ✅ Human-readable and sortable
- **Implementation**: Use atomic counter in processing thread (lock-protected)
- **Locked**: This strategy is final — no UUIDs or random IDs

### State Machine

```
┌───────────┐
│ CANDIDATE │──[frames_seen >= MIN_FRAMES_SEEN]──>┌───────────┐
│           │                                      │ CONFIRMED │──[POST /dispatch]──>┌───────────┐
│ frames: 1 │                                      │           │                      │ DISPATCHED│
│ waypoint: │                                      │ waypoint: │                      │           │
│   None    │                                      │ generated │                      │ waypoint: │
└───────────┘                                      └───────────┘                      │   exists  │
                                                                                       └───────────┘
```

**State Transitions**:

1. **CANDIDATE → CONFIRMED**:
   - **Trigger**: `frames_seen >= MIN_FRAMES_SEEN` (e.g., 3 frames)
   - **Action**: Generate waypoint file, set `confirmed_at`, emit `SURVIVOR_CONFIRMED` event
   - **Owner**: Processing thread (atomic, lock-protected)

2. **CONFIRMED → DISPATCHED**:
   - **Trigger**: POST `/api/survivors/{id}/dispatch`
   - **Action**: Set `dispatched_at`, emit `SURVIVOR_DISPATCHED` event
   - **Owner**: Processing thread (via dispatch queue)

**State Storage**:
- **In-memory**: `survivor_registry: Dict[str, Survivor]` (protected by `survivor_lock`)
- **No database**: All state is ephemeral (survives process restart only if persisted separately)

---

## 4️⃣ WebSocket Event Schema (LIVE UI)

### Event Types

#### **1. SURVIVOR_CANDIDATE**
Emitted when a new survivor candidate is detected (first frame).

```json
{
  "type": "SURVIVOR_CANDIDATE",
  "timestamp": 1234567890.123,
  "data": {
    "survivor_id": "surv_001",
    "lat": 37.7749,
    "lon": -122.4194,
    "frames_seen": 1,
    "first_seen": 1234567890.123
  }
}
```

---

#### **2. SURVIVOR_CONFIRMED**
Emitted when a candidate reaches `MIN_FRAMES_SEEN` threshold.

```json
{
  "type": "SURVIVOR_CONFIRMED",
  "timestamp": 1234567890.456,
  "data": {
    "survivor_id": "surv_001",
    "lat": 37.7749,
    "lon": -122.4194,
    "frames_seen": 3,
    "confirmed_at": 1234567890.456,
    "waypoint_file": "outputs/waypoints/survivor_001_20240101_120000.waypoints"
  }
}
```

---

#### **3. SURVIVOR_DISPATCHED**
Emitted when a confirmed survivor is marked for dispatch.

```json
{
  "type": "SURVIVOR_DISPATCHED",
  "timestamp": 1234567890.789,
  "data": {
    "survivor_id": "surv_001",
    "dispatched_at": 1234567890.789
  }
}
```

---

#### **4. TELEMETRY_UPDATE**
Emitted periodically (e.g., every N frames) with current drone position.

```json
{
  "type": "TELEMETRY_UPDATE",
  "timestamp": 1234567890.123,
  "data": {
    "frame_id": 42,
    "lat": 37.7750,
    "lon": -122.4195,
    "alt_agl": 25.5,
    "heading_deg": 180.0,
    "pitch": -90.0,
    "roll": 0.0
  }
}
```

---

### WebSocket Connection Protocol

**Endpoint**: `ws://localhost:8000/ws`

**Client → Server**:
- No messages required (server pushes events)

**Server → Client**:
- All events are JSON strings (UTF-8 encoded)
- Client should parse JSON and handle by `type` field

**Connection Lifecycle**:
1. Client connects to `/ws`
2. Server sends initial state: `{"type": "INITIAL_STATE", "survivors": [...]}`
3. Server pushes events as they occur
4. Client disconnects gracefully on page close

---

## 5️⃣ REST API Contracts

### Base URL
`http://localhost:8000/api`

---

### **GET /api/survivors**
List all survivors (all statuses).

**Response**:
```json
{
  "survivors": [
    {
      "id": "surv_001",
      "lat": 37.7749,
      "lon": -122.4194,
      "status": "CONFIRMED",
      "frames_seen": 3,
      "first_seen": 1234567890.123,
      "confirmed_at": 1234567890.456,
      "dispatched_at": null,
      "waypoint_file": "outputs/waypoints/survivor_001_20240101_120000.waypoints"
    },
    {
      "id": "surv_002",
      "lat": 37.7750,
      "lon": -122.4195,
      "status": "CANDIDATE",
      "frames_seen": 2,
      "first_seen": 1234567890.789,
      "confirmed_at": null,
      "dispatched_at": null,
      "waypoint_file": null
    }
  ],
  "count": 2
}
```

**Status Codes**:
- `200 OK`: Success

**Concurrency**: Lock-protected snapshot (read-only)

---

### **GET /api/survivors/{id}**
Get single survivor by ID.

**Path Parameters**:
- `id` (string): Survivor ID

**Response**:
```json
{
  "id": "surv_001",
  "lat": 37.7749,
  "lon": -122.4194,
  "status": "CONFIRMED",
  "frames_seen": 3,
  "first_seen": 1234567890.123,
  "confirmed_at": 1234567890.456,
  "dispatched_at": null,
  "waypoint_file": "outputs/waypoints/survivor_001_20240101_120000.waypoints"
}
```

**Status Codes**:
- `200 OK`: Success
- `404 Not Found`: Survivor not found

**Concurrency**: Lock-protected snapshot (read-only)

---

### **GET /api/survivors/{id}/waypoint**
Download waypoint file for a survivor.

**Path Parameters**:
- `id` (string): Survivor ID

**Response**:
- **Content-Type**: `text/plain` (or `application/octet-stream`)
- **Body**: Raw waypoint file content (QGC WPL 110 format)
- **Headers**: `Content-Disposition: attachment; filename="survivor_001_20240101_120000.waypoints"`

**Status Codes**:
- `200 OK`: Success
- `404 Not Found`: Survivor not found
- `400 Bad Request`: Survivor has no waypoint file (status is CANDIDATE)

**Concurrency**: Lock-protected path read, then file read (no lock)

---

### **POST /api/survivors/{id}/dispatch**
Mark a confirmed survivor as dispatched.

**Path Parameters**:
- `id` (string): Survivor ID

**Request Body**: (empty, or optional metadata)
```json
{}
```

**Response**:
```json
{
  "status": "queued",
  "survivor_id": "surv_001",
  "message": "Dispatch request queued for processing"
}
```

**Status Codes**:
- `202 Accepted`: Request queued successfully
- `404 Not Found`: Survivor not found
- `400 Bad Request`: Survivor is not in CONFIRMED status
- `409 Conflict`: Survivor already dispatched

**Concurrency**: 
- FastAPI thread: Validates existence (lock-protected read), queues request
- Processing thread: Processes dispatch queue, mutates state (lock-protected write)

**Note**: This endpoint does **NOT** mutate state directly. It queues a request that the processing thread will handle atomically.

---

## 6️⃣ Concurrency Safety Guarantees

### Why Race Conditions Cannot Occur

#### **1. Single Writer Principle**
- **Survivor state** (`survivor_registry`) has exactly **one writer**: the processing thread
- **FastAPI thread** is **read-only** (creates snapshots, never mutates)
- **TCP receiver thread** never touches survivor state

**Proof**: All state mutations occur inside `survivor_lock.acquire() ... survivor_lock.release()` blocks, and these blocks are **only** executed by the processing thread.

---

#### **2. TCP Reception Cannot Block Processing**
- **TCP receiver** runs in **separate thread** with its own blocking I/O
- **Processing loop** consumes from **bounded queue** with timeout (`queue.get(timeout=0.1)`)
- If TCP receiver blocks (network delay), processing thread continues (may get empty queue, skip iteration)
- If queue is full, TCP receiver **drops frames** (non-blocking `put_nowait()`)

**Proof**: Queue operations are **non-blocking** (with timeout) or **fail-fast** (drop on full). TCP blocking is isolated to receiver thread.

---

#### **3. UI Cannot Corrupt State**
- **FastAPI endpoints** are **read-only**:
  - `GET /api/survivors`: Creates snapshot (lock-protected), processes snapshot (no lock)
  - `GET /api/survivors/{id}`: Creates copy (lock-protected), returns copy (no lock)
  - `GET /api/survivors/{id}/waypoint`: Reads path (lock-protected), reads file (no lock)
- **POST /api/survivors/{id}/dispatch**: Queues request (no state mutation), processing thread handles it

**Proof**: FastAPI thread **never** calls `survivor_registry[id].status = ...` or any mutation. It only reads (with lock) and creates immutable copies.

---

#### **4. Lock Acquisition Order**
- **Single lock** (`survivor_lock`) protects all survivor state
- **No nested locks**: Processing thread acquires lock, does work, releases lock
- **No deadlock risk**: Only one lock, no circular dependencies

**Proof**: Lock is acquired **only** during state mutation, released immediately after. No lock is held during I/O (file writes, queue operations).

---

#### **5. Immutable Snapshots**
- **FastAPI reads** create **deep copies** (or `to_dict()` snapshots)
- Snapshot processing happens **after lock release**
- Multiple concurrent API requests can read simultaneously (each gets its own snapshot)

**Proof**: 
```python
# FastAPI thread
with survivor_lock:
    snapshot = {id: surv.to_dict() for id, surv in survivor_registry.items()}
# Lock released here
return JSONResponse(snapshot)  # Process snapshot without lock
```

---

#### **6. Event Queue Isolation**
- **WebSocket events** are pushed to **thread-safe queue** (`queue.Queue` or `collections.deque`)
- Processing thread pushes events (no lock needed, queue is thread-safe)
- FastAPI thread consumes events (no lock needed, queue is thread-safe)
- Event data is **immutable** (dicts, no shared references)
- **Overflow policy**: **Drop oldest events** when queue is full (UI needs latest state, not historical)

**Proof**: Queue operations are atomic. Event payloads are serialized (JSON-compatible dicts), no shared mutable state. Oldest-first dropping ensures UI always receives most recent state updates.

---

#### **7. Dispatch Queue Safety**
- **Dispatch requests** go through **separate queue** (`dispatch_queue`)
- FastAPI thread enqueues (non-blocking)
- Processing thread dequeues and processes (atomic, lock-protected)

**Proof**: Queue is thread-safe. Processing thread acquires lock before state mutation, ensuring atomicity.

---

### Summary: Race Condition Prevention

| Component | State Access | Lock Usage | Race Condition Risk |
|-----------|--------------|------------|---------------------|
| TCP Receiver | Queue push only | None | ✅ None (queue is thread-safe) |
| Processing Loop | Read + Write | `survivor_lock` (during mutation) | ✅ None (single writer) |
| FastAPI REST | Read-only (snapshots) | `survivor_lock` (during snapshot) | ✅ None (immutable copies) |
| FastAPI WebSocket | Event queue push | None | ✅ None (queue is thread-safe) |
| Dispatch Handler | Queue → Lock → Mutate | `survivor_lock` (during mutation) | ✅ None (atomic operation) |

---

## Implementation Notes

### Required Python Modules
- `threading`: `Lock`, `Thread`
- `queue`: `Queue` (bounded queue for frames)
- `fastapi`: REST + WebSocket server
- `uvicorn`: ASGI server
- `ultralytics`: YOLO model
- `cv2`: Image processing
- `numpy`: Array operations

### Queue Sizes
- **Frame queue**: 10 frames (configurable, drops newest on full)
- **WebSocket event queue**: 100 events (configurable, **drops oldest on full** — UI needs latest state)
- **Dispatch queue**: 50 requests (configurable, blocks if full)

### Lock Granularity
- **Coarse-grained**: Single lock protects entire `survivor_registry`
- **Fine-grained alternative**: Per-survivor locks (more complex, not necessary for current scale)

### Performance Considerations
- **Frame dropping**: Acceptable for stability
- **Lock contention**: Minimal (processing thread holds lock < 10ms per frame)
- **Snapshot overhead**: Acceptable (survivor count < 100 typically)

---

## Conclusion

This architecture ensures **provable race-condition safety** through:
1. **Strict separation of concerns** (TCP → Processing → API)
2. **Single writer principle** (processing thread owns all mutations)
3. **Immutable snapshots** (API reads never corrupt state)
4. **Thread-safe queues** (isolate blocking I/O from processing)
5. **Atomic state transitions** (lock-protected mutations)

The system is designed to scale to **live UI updates** without instability, with frame dropping as a safety mechanism to prevent queue overflow.

---

**Document Version**: 1.0  
**Last Updated**: 2024-01-01  
**Status**: Design Complete — Ready for Implementation
