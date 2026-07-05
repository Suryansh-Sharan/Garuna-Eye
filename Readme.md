# Garuna-Eye: Drone Survivor Detection and Geotagging Workspace

## 1. Project Purpose

Garuna-Eye is a drone-assisted search-and-rescue system that detects people from aerial video, estimates their GPS location, and generates waypoint files for rescue 

At a high level:

- A Raspberry Pi on the drone captures camera frames and reads Pixhawk telemetry.
- The Pi sends frame + telemetry packets to a ground laptop.
- The laptop runs YOLO person detection and geotagging math.
- Confirmed detections are exported as map markers and QGroundControl waypoint files.
- A web dashboard path exists (Version-2 frontend/backend), while a Next.js app also exists as a separate, still-boilerplate frontend track.

This repository is a multi-version workspace. It contains active code, older reference code, and experimental web/API implementations.

---

## 2. Repository Structure (What Each Top-Level Folder Is)

### `Pipeline/` (Most practical active implementation)

Primary live-flight and offline-replay workflow.

Key files:

- `tcptagger.py`:
	- Laptop-side TCP receiver.
	- Runs YOLO inference.
	- Converts detection pixel positions into lat/lon.
	- Deduplicates detections via clustering.
	- Writes annotated images, map HTML, and waypoint file.
- `tcp_sender.py`:
	- Raspberry Pi-side sender.
	- Reads Pixhawk telemetry via MAVLink.
	- Captures camera frames.
	- Always records local flight logs first, then best-effort TCP streaming.
- `offline_flight.py`:
	- Replays saved flight logs (`flights/flight_*`) and regenerates detections/maps/waypoints offline.
- `config.yaml`:
	- Core runtime settings: camera intrinsics/FOV, YOLO model path, dedup radius, geotag validation limits, TCP port.
- `how_python_files_are_runned.txt`:
	- Quick run commands for laptop and Raspberry Pi.

Supporting data folders:

- `flights/`: Recorded flight sessions (frames + telemetry log).
- `outputs/`: Generated images/maps/waypoints/final_map.
- `waypoints/`: Mission files produced over time.

### `Version-2/` (Architecture-focused track, includes web system)

Production-style architecture and API/UI split.

Key files:

- `ARCHITECTURE_DESIGN.md`:
	- Clear design for race-condition-safe multi-threading.
	- Defines producer-consumer queues, single-writer processing model, lock-protected state.
	- Documents thread responsibilities and forbidden actions per thread.
- `backend_skeleton.py`:
	- FastAPI backend with thread-safe survivor registry and queue orchestration.
	- Includes TCP receiver + processing + WebSocket/REST integration in one service.
	- This file has significant structure in place, but still contains TODOs for full processing parity.
- `BACKEND_SKELETON_README.md`:
	- API and backend usage notes.
- `frontend/`:
	- React app for survivor list/details/dispatch.
	- Uses REST + WebSocket updates.
	- Main app entry: `frontend/src/App.js`.

### `Person-Detection-and-GeoTagging-master/` (Legacy/reference prototype)

Earlier full prototype with setup and calibration scripts.

Contains:

- Setup docs (`Readme.md`, `QUICKSTART_CHECKLIST.md`, `Testing_guide.md`).
- Geotagging variants (`maingeo.py`, `geo*.py`, `tcptagger.py`).
- Camera calibration tools (`camera_calibration.py`, `fisheye_calibration.py`, `camera_calibration.yaml`).
- Sender variants (`raspberry_pi_sender.py`, `raspberry_pi_sender2.py`, `rasp_sender.py`).
- Model/training scripts (`train.py`, weights like `yolo11n.pt`).

Useful as historical reference and calibration/test toolkit.

### `backend/` (Experimental early FastAPI backend)

Contains:

- `main.py`: FastAPI app with frame ingestion endpoints, GPS UDP listener, stream endpoints, and websocket broadcast.
- `detect.py`: UDP frame receiver + YOLO inference path that posts results to backend endpoints.

This path appears experimental and separate from the `Pipeline/` primary flow.

### `garuna-eye/` (Next.js frontend scaffold)

Next.js app generated via create-next-app.

- Current `app/page.tsx` is starter template UI.
- `README.md` is default Next.js boilerplate.

This is currently not integrated into the mission workflow.

### `frontend-folder/`

Currently empty placeholder.

### `Pics/`

Contains hardware/view reference images (camera/drone orientation visuals).

---

## 3. System Data Flow (Operational View)

### Live Flight Path

1. Raspberry Pi (`Pipeline/tcp_sender.py`)
	 - Reads telemetry from Pixhawk (`GLOBAL_POSITION_INT`, `ATTITUDE`).
	 - Captures camera frame and compresses JPEG.
	 - Saves frame + telemetry locally to `flights/flight_<timestamp>/`.
	 - Sends TCP payload (JSON metadata + JPEG) to laptop when available.

2. Laptop (`Pipeline/tcptagger.py`)
	 - Receives TCP packet (4-byte size header + payload).
	 - Parses metadata into drone state.
	 - Runs YOLO person detection.
	 - Computes world ray from detection pixel and converts to ground offset.
	 - Transforms offset to latitude/longitude.
	 - Applies max geotag distance and cluster-based dedup.
	 - Writes outputs:
		 - Annotated image in `outputs/images/`
		 - Survivor map in `outputs/maps/`
		 - Waypoints in `outputs/waypoints/`

3. Optional command center integration
	 - `Version-2/backend_skeleton.py` + `Version-2/frontend/` can provide REST/WebSocket-based monitoring and dispatch workflows.

### Offline Replay Path

`Pipeline/offline_flight.py` reprocesses historical flights from `flights/flight_*` to regenerate detections, maps, and waypoints without live network/hardware dependency.

---

## 4. Typical Run Commands

## 4.1 Primary (Pipeline) Live Mode

Laptop:

```bash
cd Pipeline
python tcptagger.py --config config.yaml
```

Raspberry Pi:

```bash
cd Pipeline
python tcp_sender.py --laptop-ip <LAPTOP_IP> --port 7000 --fps 1
```

## 4.2 Offline Replay

```bash
cd Pipeline
python offline_flight.py --flight-dir flights/flight_<timestamp> --config config.yaml
```

## 4.3 Version-2 Backend + Frontend

Backend:

```bash
cd Version-2
pip install -r requirements.txt
python backend_skeleton.py
```

Frontend:

```bash
cd Version-2/frontend
npm install
npm start
```

## 4.4 Experimental Backend Track

```bash
cd backend
python main.py
```

---

## 5. Important Configuration Knobs

From `Pipeline/config.yaml`:

- `camera.camera_matrix` / `camera.distortion_coeffs`: Camera intrinsics used for geometric accuracy.
- `camera.hfov_deg`, `camera.vfov_deg`: Angular mapping from pixels to rays.
- `yolo.model_path`, `yolo.confidence_threshold`, `yolo.imgsz`: Detector behavior.
- `deduplication.min_distance_meters`: Survivor clustering radius.
- `gimbal.pitch_offset_deg`: Corrects camera mounting angle.
- `validation.max_geotag_distance`: Rejects unrealistic projected detections.
- `network.tcp_port`: Receiver/sender communication port.

---

## 6. Current Reality of the Workspace (What Is Complete vs Partial)

Most complete and directly useful path:

- `Pipeline/` for live and offline detection+geotagging.

Well-documented architecture with growing implementation:

- `Version-2/` (especially threading/API design).

Legacy but still useful for calibration/testing references:

- `Person-Detection-and-GeoTagging-master/`.

Experimental or not yet integrated:

- `backend/` (separate prototype backend flow).
- `garuna-eye/` (starter Next.js app).
- `frontend-folder/` (empty).

---

## 7. Suggested Consolidation Strategy

If you want a cleaner production repository:

1. Keep `Pipeline/` as immediate operational baseline.
2. Merge selected `Version-2/` concurrency + API patterns into active runtime.
3. Choose one frontend path (`Version-2/frontend` React or `garuna-eye` Next.js) and retire the other.
4. Move legacy prototype (`Person-Detection-and-GeoTagging-master/`) to an `archive/` folder with clear status.
5. Keep calibration and testing tools, but centralize docs and configs to one canonical source.

---

## 8. Quick Folder Status Matrix

| Folder | Role | Status |
|---|---|---|
| `Pipeline/` | Live + offline mission processing | Active |
| `Version-2/` | Thread-safe architecture + API/UI track | Active (partially implemented runtime) |
| `Person-Detection-and-GeoTagging-master/` | Legacy full prototype and tooling | Reference |
| `backend/` | Early experimental FastAPI + detect pipeline | Experimental |
| `garuna-eye/` | Next.js scaffold | Boilerplate |
| `frontend-folder/` | Placeholder | Empty |
| `Pics/` | Reference images | Data |

---

## 9. In One Line

This workspace is a real-world drone SAR codebase with an active TCP+YOLO+geotagging pipeline, a strong next-gen architecture draft, and multiple legacy/experimental branches that should be consolidated around one canonical runtime and UI.
