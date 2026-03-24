import { useState, useEffect, useRef } from 'react';

export function useWebSocket() {
  const [events, setEvents] = useState([]);
  const [status, setStatus] = useState('disconnected');
  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);
  const maxReconnectAttempts = 5;
  const reconnectDelay = 3000; // 3 seconds

  useEffect(() => {
    const connect = () => {
      // Don't reconnect if already connected or max attempts reached
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        return;
      }

      if (reconnectAttemptsRef.current >= maxReconnectAttempts) {
        console.error('Max reconnection attempts reached');
        setStatus('failed');
        return;
      }

      try {
        // Determine WebSocket URL
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = process.env.REACT_APP_WS_URL || `${protocol}//${window.location.hostname}:8000/ws`;
        
        console.log('Connecting to WebSocket:', wsUrl);
        setStatus('connecting');

        const ws = new WebSocket(wsUrl);

        ws.onopen = () => {
          console.log('WebSocket connected');
          setStatus('connected');
          reconnectAttemptsRef.current = 0;
          
          // Clear any pending reconnect timeout
          if (reconnectTimeoutRef.current) {
            clearTimeout(reconnectTimeoutRef.current);
            reconnectTimeoutRef.current = null;
          }
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            
            // Handle initial state
            if (data.type === 'INITIAL_STATE') {
              setEvents(prev => [...prev, data]);
            } else {
              // Handle real-time events
              setEvents(prev => [...prev, data]);
            }
          } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
          }
        };

        ws.onerror = (error) => {
          console.error('WebSocket error:', error);
          setStatus('error');
        };

        ws.onclose = () => {
          console.log('WebSocket disconnected');
          setStatus('disconnected');
          wsRef.current = null;

          // Attempt to reconnect
          if (reconnectAttemptsRef.current < maxReconnectAttempts) {
            reconnectAttemptsRef.current += 1;
            console.log(`Reconnecting in ${reconnectDelay}ms (attempt ${reconnectAttemptsRef.current}/${maxReconnectAttempts})`);
            
            reconnectTimeoutRef.current = setTimeout(() => {
              connect();
            }, reconnectDelay);
          } else {
            setStatus('failed');
          }
        };

        wsRef.current = ws;
      } catch (error) {
        console.error('Failed to create WebSocket:', error);
        setStatus('error');
      }
    };

    connect();

    // Cleanup on unmount
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, []);

  return { events, status };
}
