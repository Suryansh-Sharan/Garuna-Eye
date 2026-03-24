# Backend Skeleton - Race-Condition-Safe Drone Ground Control System

## Overview

This is a **complete runnable backend skeleton** that implements the architecture design with:
- ✅ Producer-consumer queues (thread-safe)
- ✅ Single processing owner (lock-protected state mutations)
- ✅ FastAPI (REST + WebSocket)
- ✅ Thread-safe survivor state management

## Current Status: STUBS ONLY

- ❌ **NO YOLO** - Placeholder comments mark where YOLO inference will be added
- ❌ **NO OpenCV** - Frame processing stubs only
- ❌ **NO TCP logic** - TCP receiver thread simulates frames with logging

## Architecture

### Threads

1. **TCP Receiver Thread** (`tcp_receiver_thread`)
   - STUB: Simulates receiving frames every 2 seconds
   - Pushes to `frame_queue` (drops if full)

2. **Processing Loop Thread** (`processing_loop_thread`)
   - STUB: Consumes frames, simulates survivor detection
   - Single owner of all state mutations
   - Emits WebSocket events

3. **FastAPI/Uvicorn** (Uvicorn's event loop)
   - REST endpoints (read-only state access)
   - WebSocket endpoint (`/ws`)
   - Background task for event broadcasting

### Queues

- `frame_queue`: TCP Receiver → Processing Loop (maxsize=10)
- `websocket_event_queue`: Processing Loop → FastAPI (maxsize=100, drops oldest)
- `dispatch_queue`: FastAPI → Processing Loop (maxsize=50)

### State Management

- `SurvivorManager`: Lock-protected in-memory survivor registry
- Sequential IDs: `surv_001`, `surv_002`, `surv_003`, ...
- State transitions: `CANDIDATE → CONFIRMED → DISPATCHED`

## Installation

```bash
pip install -r requirements.txt
```

## Running

```bash
python backend_skeleton.py
```

Server starts on `http://0.0.0.0:8000`

## API Endpoints

### REST API

- `GET /api/survivors` - List all survivors
- `GET /api/survivors/{id}` - Get single survivor
- `GET /api/survivors/{id}/waypoint` - Download waypoint file
- `POST /api/survivors/{id}/dispatch` - Queue dispatch request

### WebSocket

- `ws://localhost:8000/ws` - Live event stream

## Testing

### Test REST API

```bash
# List survivors
curl http://localhost:8000/api/survivors

# Get single survivor
curl http://localhost:8000/api/survivors/surv_001

# Dispatch survivor (must be CONFIRMED)
curl -X POST http://localhost:8000/api/survivors/surv_001/dispatch
```

### Test WebSocket

Use a WebSocket client (e.g., `websocat`, browser console):

```javascript
const ws = new WebSocket('ws://localhost:8000/ws');
ws.onmessage = (event) => {
    console.log('Event:', JSON.parse(event.data));
};
```

## Implementation Notes

### Where to Add Real Logic

1. **TCP Receiver Thread** (`tcp_receiver_thread` function):
   - Replace stub with actual TCP socket connection
   - Add packet decoding (JSON + JPEG)
   - Add `cv2.imdecode()` for image decoding

2. **Processing Loop Thread** (`processing_loop_thread` function):
   - Add frame validation (pitch/roll check)
   - Add frame stabilization (ORB + RANSAC)
   - Add YOLO inference (`model.predict()`)
   - Add geolocation computation (pixel → lat/lon)
   - Add waypoint file generation

3. **Survivor Confirmation**:
   - Implement spatial clustering logic
   - Add temporal confirmation (MIN_FRAMES_SEEN threshold)
   - Generate waypoint files on confirmation

### Concurrency Safety

- ✅ All state mutations are lock-protected
- ✅ FastAPI endpoints are read-only (create snapshots)
- ✅ Queues are thread-safe
- ✅ Single writer principle enforced (processing thread only)

## Design Compliance

This skeleton strictly follows `ARCHITECTURE_DESIGN.md`:
- ✅ Producer-consumer model
- ✅ Single processing owner
- ✅ Thread-safe state access
- ✅ FastAPI read-only pattern
- ✅ Sequential survivor IDs
- ✅ Event queue drops oldest when full

## Next Steps

1. Implement TCP receiver logic
2. Add YOLO model loading and inference
3. Implement frame stabilization
4. Add geolocation computation
5. Implement waypoint file generation
6. Add configuration file support
