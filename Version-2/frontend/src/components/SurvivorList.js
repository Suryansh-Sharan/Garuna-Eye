import React from 'react';
import './SurvivorList.css';

function SurvivorList({ survivors, selectedId, onSelect }) {
  const getStatusColor = (status) => {
    switch (status) {
      case 'CANDIDATE':
        return '#f39c12'; // Orange
      case 'CONFIRMED':
        return '#3498db'; // Blue
      case 'DISPATCHED':
        return '#27ae60'; // Green
      default:
        return '#95a5a6'; // Gray
    }
  };

  const formatTimestamp = (timestamp) => {
    if (!timestamp) return 'N/A';
    const date = new Date(timestamp * 1000);
    return date.toLocaleTimeString();
  };

  return (
    <div className="survivor-list">
      <div className="survivor-list-header">
        <h2>Survivors ({survivors.length})</h2>
      </div>

      {survivors.length === 0 ? (
        <div className="empty-list">
          <p>No survivors detected yet</p>
        </div>
      ) : (
        <div className="survivor-items">
          {survivors.map(survivor => (
            <div
              key={survivor.id}
              className={`survivor-item ${selectedId === survivor.id ? 'selected' : ''}`}
              onClick={() => onSelect(survivor.id)}
            >
              <div className="survivor-item-header">
                <span className="survivor-id">{survivor.id}</span>
                <span
                  className="status-badge"
                  style={{ backgroundColor: getStatusColor(survivor.status) }}
                >
                  {survivor.status}
                </span>
              </div>

              <div className="survivor-item-details">
                <div className="detail-row">
                  <span className="detail-label">Location:</span>
                  <span className="detail-value">
                    {survivor.lat.toFixed(6)}, {survivor.lon.toFixed(6)}
                  </span>
                </div>
                <div className="detail-row">
                  <span className="detail-label">Frames Seen:</span>
                  <span className="detail-value">{survivor.frames_seen}</span>
                </div>
                {survivor.confirmed_at && (
                  <div className="detail-row">
                    <span className="detail-label">Confirmed:</span>
                    <span className="detail-value">{formatTimestamp(survivor.confirmed_at)}</span>
                  </div>
                )}
                {survivor.dispatched_at && (
                  <div className="detail-row">
                    <span className="detail-label">Dispatched:</span>
                    <span className="detail-value">{formatTimestamp(survivor.dispatched_at)}</span>
                  </div>
                )}
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

export default SurvivorList;
