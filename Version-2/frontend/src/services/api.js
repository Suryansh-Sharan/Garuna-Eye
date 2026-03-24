const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api';

/**
 * Fetch all survivors
 */
export async function fetchSurvivors() {
  const response = await fetch(`${API_BASE_URL}/survivors`);
  
  if (!response.ok) {
    throw new Error(`Failed to fetch survivors: ${response.statusText}`);
  }
  
  return response.json();
}

/**
 * Fetch a single survivor by ID
 */
export async function fetchSurvivor(survivorId) {
  const response = await fetch(`${API_BASE_URL}/survivors/${survivorId}`);
  
  if (!response.ok) {
    if (response.status === 404) {
      throw new Error('Survivor not found');
    }
    throw new Error(`Failed to fetch survivor: ${response.statusText}`);
  }
  
  return response.json();
}

/**
 * Download waypoint file for a survivor
 */
export async function downloadWaypoint(survivorId) {
  const response = await fetch(`${API_BASE_URL}/survivors/${survivorId}/waypoint`);
  
  if (!response.ok) {
    if (response.status === 404) {
      throw new Error('Waypoint file not found');
    }
    if (response.status === 400) {
      throw new Error('Survivor has no waypoint file');
    }
    throw new Error(`Failed to download waypoint: ${response.statusText}`);
  }
  
  // Get filename from Content-Disposition header or use default
  const contentDisposition = response.headers.get('Content-Disposition');
  let filename = `survivor_${survivorId}.waypoints`;
  
  if (contentDisposition) {
    const filenameMatch = contentDisposition.match(/filename="?(.+?)"?$/);
    if (filenameMatch) {
      filename = filenameMatch[1];
    }
  }
  
  // Create blob and download
  const blob = await response.blob();
  const url = window.URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  window.URL.revokeObjectURL(url);
  document.body.removeChild(a);
}

/**
 * Dispatch a survivor (POST /api/survivors/{id}/dispatch)
 */
export async function dispatchSurvivor(survivorId) {
  const response = await fetch(`${API_BASE_URL}/survivors/${survivorId}/dispatch`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
  });
  
  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    const errorMessage = errorData.detail || `Failed to dispatch survivor: ${response.statusText}`;
    throw new Error(errorMessage);
  }
  
  return response.json();
}
