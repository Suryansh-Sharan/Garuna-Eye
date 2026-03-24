# Quick Start Guide

## Prerequisites

- Node.js 16+ and npm installed
- Backend running on `http://localhost:8000`

## Setup

1. Install dependencies:
```bash
cd frontend
npm install
```

2. Start the development server:
```bash
npm start
```

3. Open browser to `http://localhost:3000`

## Features Overview

### Survivor List
- Shows all detected survivors with their status
- Color-coded status badges:
  - 🟠 CANDIDATE (orange)
  - 🔵 CONFIRMED (blue)
  - 🟢 DISPATCHED (green)
- Click a survivor to view details

### Survivor Details
- View complete survivor information
- Download waypoint file (for CONFIRMED/DISPATCHED survivors)
- Dispatch button (for CONFIRMED survivors only)

### Real-time Updates
- WebSocket connection indicator in header
- Automatic updates when survivors change status
- No page refresh needed

## Usage

1. **View Survivors**: The list automatically updates as survivors are detected
2. **Select Survivor**: Click on any survivor in the list to view details
3. **Download Waypoint**: Click "Download Waypoint File" to save the `.waypoints` file
4. **Dispatch**: Click "Dispatch for Mission Planner" to mark survivor as dispatched

## Troubleshooting

### WebSocket Connection Issues
- Check that backend is running on port 8000
- Verify CORS settings in backend
- Check browser console for WebSocket errors

### API Errors
- Ensure backend is running: `python backend_skeleton.py`
- Check backend logs for errors
- Verify API endpoints are accessible

### Build Issues
- Clear `node_modules` and reinstall: `rm -rf node_modules && npm install`
- Check Node.js version: `node --version` (should be 16+)
