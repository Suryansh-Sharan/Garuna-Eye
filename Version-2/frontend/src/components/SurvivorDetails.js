import React, { useState } from 'react';
import './SurvivorDetails.css';
import { downloadWaypoint, dispatchSurvivor } from '../services/api';

function SurvivorDetails({ survivor, onDeselect }) {
  const [isDispatching, setIsDispatching] = useState(false);
  const [dispatchError, setDispatchError] = useState(null);
  const [dispatchSuccess, setDispatchSuccess] = useState(false);

  const handleDownloadWaypoint = async () => {
    if (!survivor.waypoint_file) {
      alert('No waypoint file available for this survivor');
      return;
    }

    try {
      await downloadWaypoint(survivor.id);
    } catch (error) {
      console.error('Failed to download waypoint:', error);
      alert('Failed to download waypoint file');
    }
  };

  const handleDispatch = async () => {
    if (survivor.status !== 'CONFIRMED') {
      alert('Only CONFIRMED survivors can be dispatched');
      return;
    }

    setIsDispatching(true);
    setDispatchError(null);
    setDispatchSuccess(false);

    try {
      await dispatchSurvivor(survivor.id);
      setDispatchSuccess(true);
      setTimeout(() => setDispatchSuccess(false), 3000);
    } catch (error) {
      console.error('Failed to dispatch survivor:', error);
      setDispatchError(error.message || 'Failed to dispatch survivor');
    } finally {
      setIsDispatching(false);
    }
  };

  const formatTimestamp = (timestamp) => {
    if (!timestamp) return 'N/A';
    const date = new Date(timestamp * 1000);
    return date.toLocaleString();
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'CANDIDATE':
        return '#f39c12';
      case 'CONFIRMED':
        return '#3498db';
      case 'DISPATCHED':
        return '#27ae60';
      default:
        return '#95a5a6';
    }
  };

  return (
    <div className="survivor-details">
      <div className="survivor-details-header">
        <h2>Survivor Details</h2>
        <button className="close-button" onClick={onDeselect}>
          ×
        </button>
      </div>

      <div className="survivor-details-content">
        <div className="detail-section">
          <div className="detail-group">
            <span className="detail-label">ID</span>
            <span className="detail-value">{survivor.id}</span>
          </div>

          <div className="detail-group">
            <span className="detail-label">Status</span>
            <span
              className="status-badge"
              style={{ backgroundColor: getStatusColor(survivor.status) }}
            >
              {survivor.status}
            </span>
          </div>
        </div>

        <div className="detail-section">
          <h3>Location</h3>
          <div className="detail-group">
            <span className="detail-label">Latitude</span>
            <span className="detail-value">{survivor.lat.toFixed(7)}</span>
          </div>
          <div className="detail-group">
            <span className="detail-label">Longitude</span>
            <span className="detail-value">{survivor.lon.toFixed(7)}</span>
          </div>
        </div>

        <div className="detail-section">
          <h3>Detection Info</h3>
          <div className="detail-group">
            <span className="detail-label">Frames Seen</span>
            <span className="detail-value">{survivor.frames_seen}</span>
          </div>
          <div className="detail-group">
            <span className="detail-label">First Seen</span>
            <span className="detail-value">{formatTimestamp(survivor.first_seen)}</span>
          </div>
          {survivor.confirmed_at && (
            <div className="detail-group">
              <span className="detail-label">Confirmed At</span>
              <span className="detail-value">{formatTimestamp(survivor.confirmed_at)}</span>
            </div>
          )}
          {survivor.dispatched_at && (
            <div className="detail-group">
              <span className="detail-label">Dispatched At</span>
              <span className="detail-value">{formatTimestamp(survivor.dispatched_at)}</span>
            </div>
          )}
        </div>

        {survivor.waypoint_file && (
          <div className="detail-section">
            <h3>Waypoint File</h3>
            <div className="detail-group">
              <span className="detail-label">File</span>
              <span className="detail-value file-path">{survivor.waypoint_file}</span>
            </div>
          </div>
        )}

        <div className="action-buttons">
          {survivor.waypoint_file && (
            <button
              className="action-button download-button"
              onClick={handleDownloadWaypoint}
            >
              Download Waypoint File
            </button>
          )}

          {survivor.status === 'CONFIRMED' && (
            <button
              className="action-button dispatch-button"
              onClick={handleDispatch}
              disabled={isDispatching}
            >
              {isDispatching ? 'Dispatching...' : 'Dispatch for Mission Planner'}
            </button>
          )}

          {survivor.status === 'CANDIDATE' && (
            <div className="info-message">
              Survivor must be CONFIRMED before dispatch
            </div>
          )}

          {survivor.status === 'DISPATCHED' && (
            <div className="success-message">
              Survivor has been dispatched
            </div>
          )}
        </div>

        {dispatchError && (
          <div className="error-message">
            {dispatchError}
          </div>
        )}

        {dispatchSuccess && (
          <div className="success-message">
            Survivor dispatched successfully! Waypoint file ready for Mission Planner upload.
          </div>
        )}
      </div>
    </div>
  );
}

export default SurvivorDetails;
