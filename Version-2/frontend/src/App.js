import React, { useState, useEffect } from 'react';
import './App.css';
import SurvivorList from './components/SurvivorList';
import SurvivorDetails from './components/SurvivorDetails';
import { useWebSocket } from './hooks/useWebSocket';
import { fetchSurvivors } from './services/api';

function App() {
  const [survivors, setSurvivors] = useState([]);
  const [selectedSurvivorId, setSelectedSurvivorId] = useState(null);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');

  // WebSocket connection for real-time updates
  const { events, status } = useWebSocket();

  // Update connection status
  useEffect(() => {
    setConnectionStatus(status);
  }, [status]);

  // Handle WebSocket events
  useEffect(() => {
    if (!events.length) return;

    const latestEvent = events[events.length - 1];

    switch (latestEvent.type) {
      case 'INITIAL_STATE':
        setSurvivors(latestEvent.survivors || []);
        break;

      case 'SURVIVOR_CANDIDATE':
        setSurvivors(prev => {
          const exists = prev.find(s => s.id === latestEvent.data.survivor_id);
          if (exists) return prev;
          return [...prev, {
            id: latestEvent.data.survivor_id,
            ...latestEvent.data,
            status: 'CANDIDATE',
            waypoint_file: null,
            confirmed_at: null,
            dispatched_at: null
          }];
        });
        break;

      case 'SURVIVOR_CONFIRMED':
        setSurvivors(prev => prev.map(s =>
          s.id === latestEvent.data.survivor_id
            ? { ...s, ...latestEvent.data, status: 'CONFIRMED' }
            : s
        ));
        break;

      case 'SURVIVOR_DISPATCHED':
        setSurvivors(prev => prev.map(s =>
          s.id === latestEvent.data.survivor_id
            ? { ...s, status: 'DISPATCHED', dispatched_at: latestEvent.data.dispatched_at }
            : s
        ));
        break;

      default:
        break;
    }
  }, [events]);

  // Initial fetch of survivors
  useEffect(() => {
    const loadSurvivors = async () => {
      try {
        const data = await fetchSurvivors();
        setSurvivors(data.survivors || []);
      } catch (error) {
        console.error('Failed to load survivors:', error);
      }
    };

    loadSurvivors();
  }, []);

  const selectedSurvivor = survivors.find(s => s.id === selectedSurvivorId);

  return (
    <div className="app">
      <header className="app-header">
        <h1>Drone Ground Control System</h1>
        <div className="connection-status">
          <span className={`status-indicator ${connectionStatus}`}></span>
          <span>{connectionStatus === 'connected' ? 'Connected' : 'Disconnected'}</span>
        </div>
      </header>

      <main className="app-main">
        <div className="survivor-list-container">
          <SurvivorList
            survivors={survivors}
            selectedId={selectedSurvivorId}
            onSelect={setSelectedSurvivorId}
          />
        </div>

        <div className="survivor-details-container">
          {selectedSurvivor ? (
            <SurvivorDetails
              survivor={selectedSurvivor}
              onDeselect={() => setSelectedSurvivorId(null)}
            />
          ) : (
            <div className="no-selection">
              <p>Select a survivor to view details</p>
            </div>
          )}
        </div>
      </main>
    </div>
  );
}

export default App;
