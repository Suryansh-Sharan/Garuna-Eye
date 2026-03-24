# Drone Ground Control System - Frontend

Minimal React frontend for the Drone Ground Control System backend.

## Features

- **Live Survivor List**: Real-time list of detected survivors with status (CANDIDATE / CONFIRMED / DISPATCHED)
- **WebSocket Integration**: Real-time updates via WebSocket connection
- **Survivor Details**: View detailed information about selected survivors
- **Waypoint Download**: Download waypoint files for Mission Planner upload
- **Dispatch Support**: Trigger dispatch for confirmed survivors

## Installation

```bash
cd frontend
npm install
```

## Development

```bash
npm start
```

The app will start on `http://localhost:3000` and proxy API requests to `http://localhost:8000`.

## Environment Variables

Create a `.env` file in the `frontend` directory:

```env
REACT_APP_API_URL=http://localhost:8000/api
REACT_APP_WS_URL=ws://localhost:8000/ws
```

If not set, defaults to:
- API: `http://localhost:8000/api`
- WebSocket: `ws://localhost:8000/ws` (or `wss://` for HTTPS)

## Production Build

```bash
npm run build
```

The build output will be in the `build` directory.

## Architecture

- **React Hooks**: Uses `useState` and `useEffect` for state management
- **WebSocket Hook**: Custom `useWebSocket` hook for real-time updates
- **API Service**: Centralized API functions in `services/api.js`
- **Component Structure**:
  - `App.js`: Main application component
  - `SurvivorList.js`: List of survivors
  - `SurvivorDetails.js`: Detailed view and actions

## Backend Integration

The frontend connects to the backend via:
- **REST API**: `GET /api/survivors`, `GET /api/survivors/{id}`, `GET /api/survivors/{id}/waypoint`, `POST /api/survivors/{id}/dispatch`
- **WebSocket**: `ws://localhost:8000/ws` for real-time events

## WebSocket Events

The frontend handles the following WebSocket events:
- `INITIAL_STATE`: Initial survivor list on connection
- `SURVIVOR_CANDIDATE`: New survivor candidate detected
- `SURVIVOR_CONFIRMED`: Survivor confirmed (waypoint file generated)
- `SURVIVOR_DISPATCHED`: Survivor dispatched

## Notes

- Frontend is read-only with respect to backend concurrency (no state mutations)
- All state changes happen via WebSocket events from backend
- No map rendering (list-based UI only)
