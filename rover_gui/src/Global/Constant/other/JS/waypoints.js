class Waypoint {
  #waypointCounter = 1;
  #activeWaypoint = null;
  #isAddingWaypoint = false;

    constructor(viewer) 
    {
        this.viewer = viewer;
    }

    clearAllWaypoints() 
    {
        waypointEntities.forEach(waypoint => {
        viewer.entities.remove(waypoint);
  });
  waypointEntities = [];
  waypointCounter = 1;

  if (pathEntity) {
    viewer.entities.remove(pathEntity);
    pathEntity = null;
  }
    }

 deleteWaypoint(waypointId) {
  const entity = viewer.entities.getById(waypointId);
  if (entity) {
    viewer.entities.remove(entity);
    waypointEntities = waypointEntities.filter(wp => wp.id !== waypointId);
    return true;
  }
  return false;
}

 drawPath(startLat, startLon, endLat, endLon) {
  if (pathEntity) {
    viewer.entities.remove(pathEntity);
  }

  pathEntity = viewer.entities.add({
    name: "Path to Waypoint",
    polyline: {
      positions: Cesium.Cartesian3.fromDegreesArray([startLon, startLat, endLon, endLat]),
      width: 3,
      material: new Cesium.PolylineOutlineMaterialProperty({
        color: Cesium.Color.YELLOW,
        outlineWidth: 1,
        outlineColor: Cesium.Color.BLACK
      }),
      clampToGround: true
    }
  });

  const distance = calculateHaversineDistance(startLat, startLon, endLat, endLon);
  if (window.bridge) {
    window.bridge.pathDistanceCalculated(distance);
  }
  return distance;
}

 startDynamicPathUpdates(destLat, destLon, waypointId) {
  activeWaypoint = {
    latitude: destLat,
    longitude: destLon,
    id: waypointId
  };

  clearInterval(pathUpdateInterval);
  pathUpdateInterval = setInterval(() => {
    if (currentPosition && activeWaypoint) {
      drawPath(
        currentPosition.latitude,
        currentPosition.longitude,
        activeWaypoint.latitude,
        activeWaypoint.longitude
      );
    }
  }, 1000);

  drawPath(
    currentPosition.latitude,
    currentPosition.longitude,
    destLat,
    destLon
  );
}

 stopDynamicPathUpdates() {
  clearInterval(pathUpdateInterval);
  pathUpdateInterval = null;
  activeWaypoint = null;

  if (pathEntity) {
    viewer.entities.remove(pathEntity);
    pathEntity = null;
  }
}

 calculateHaversineDistance(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;

  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon / 2) ** 2;

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

 addWaypoint(lat, lon, name, waypointId) {
  const waypointName = name || `Waypoint ${waypointCounter++}`;
  const id = waypointId || `waypoint_${Date.now()}`;

  const existingNameWaypoint = waypointEntities.find(wp => wp.name === waypointName);
  if (existingNameWaypoint) {
    console.log("Waypoint with name already exists:", waypointName);
    return null;
  }

  const waypointEntity = viewer.entities.add({
    id: id,
    name: waypointName,
    position: Cesium.Cartesian3.fromDegrees(lon, lat),
    point: {
      pixelSize: 10,
      color: Cesium.Color.BLUE,
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      disableDepthTestDistance: Number.POSITIVE_INFINITY
    },
    label: {
      text: waypointName,
      font: '14pt sans-serif',
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      outlineWidth: 2,
      verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
      pixelOffset: new Cesium.Cartesian2(0, -10),
      fillColor: Cesium.Color.WHITE,
      outlineColor: Cesium.Color.BLACK,
      showBackground: true,
      backgroundColor: new Cesium.Color(0.165, 0.165, 0.165, 0.7),
      heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      disableDepthTestDistance: Number.POSITIVE_INFINITY
    }
  });

  if (waypointEntity._visualizers && waypointEntity._visualizers.length > 0) {
    waypointEntity._visualizers.forEach(visualizer => {
      if (visualizer && visualizer.visualizersByDisplayID) {
        for (const displayID in visualizer.visualizersByDisplayID) {
          if (visualizer.visualizersByDisplayID[displayID]) {
            visualizer.visualizersByDisplayID[displayID]._zIndex = 999;
          }
        }
      }
    });
  }

  waypointEntities.push(waypointEntity);
  return waypointEntity;
}

showWaypointDialog(lat, lon) {
  Swal.fire({
    title: 'Add Waypoint',
    input: 'text',
    inputLabel: 'Waypoint Name',
    inputValue: `Waypoint ${waypointCounter}`,
    showCancelButton: true,
    confirmButtonText: 'Add',
    cancelButtonText: 'Cancel',
    inputValidator: (value) => {
      if (!value) return 'Please enter a name';

      const existingNameWaypoint = waypointEntities.find(wp => wp.name === value);
      if (existingNameWaypoint) {
        return 'A waypoint with this name already exists. Please choose a different name.';
      }
    }
  }).then((result) => {
    if (result.isConfirmed) {
      const name = result.value;
      const id = `waypoint_${Date.now()}`;

      const existingNameWaypoint = waypointEntities.find(wp => wp.name === name);
      if (existingNameWaypoint) {
        Swal.fire({
          title: 'Error',
          text: 'A waypoint with this name already exists. Please try again with a different name.',
          icon: 'error'
        });
        return;
      }

      const waypoint = addWaypoint(lat, lon, name, id);

      if (waypoint && window.bridge && window.bridge.waypointCreated) {
        window.bridge.waypointCreated(name, lat, lon, id);
      }
    }
  });
}
}

function clearAllWaypoints() {
  waypointEntities.forEach(waypoint => {
    viewer.entities.remove(waypoint);
  });
  waypointEntities = [];
  waypointCounter = 1;

  if (pathEntity) {
    viewer.entities.remove(pathEntity);
    pathEntity = null;
  }
}

function deleteWaypoint(waypointId) {
  const entity = viewer.entities.getById(waypointId);
  if (entity) {
    viewer.entities.remove(entity);
    waypointEntities = waypointEntities.filter(wp => wp.id !== waypointId);
    return true;
  }
  return false;
}

function drawPath(startLat, startLon, endLat, endLon) {
  if (pathEntity) {
    viewer.entities.remove(pathEntity);
  }

  pathEntity = viewer.entities.add({
    name: "Path to Waypoint",
    polyline: {
      positions: Cesium.Cartesian3.fromDegreesArray([startLon, startLat, endLon, endLat]),
      width: 3,
      material: new Cesium.PolylineOutlineMaterialProperty({
        color: Cesium.Color.YELLOW,
        outlineWidth: 1,
        outlineColor: Cesium.Color.BLACK
      }),
      clampToGround: true
    }
  });

  const distance = calculateHaversineDistance(startLat, startLon, endLat, endLon);
  if (window.bridge) {
    window.bridge.pathDistanceCalculated(distance);
  }
  return distance;
}

function startDynamicPathUpdates(destLat, destLon, waypointId) {
  activeWaypoint = {
    latitude: destLat,
    longitude: destLon,
    id: waypointId
  };

  clearInterval(pathUpdateInterval);
  pathUpdateInterval = setInterval(() => {
    if (currentPosition && activeWaypoint) {
      drawPath(
        currentPosition.latitude,
        currentPosition.longitude,
        activeWaypoint.latitude,
        activeWaypoint.longitude
      );
    }
  }, 1000);

  drawPath(
    currentPosition.latitude,
    currentPosition.longitude,
    destLat,
    destLon
  );
}

function stopDynamicPathUpdates() {
  clearInterval(pathUpdateInterval);
  pathUpdateInterval = null;
  activeWaypoint = null;

  if (pathEntity) {
    viewer.entities.remove(pathEntity);
    pathEntity = null;
  }
}

function calculateHaversineDistance(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;

  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon / 2) ** 2;

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c;
}

function addWaypoint(lat, lon, name, waypointId) {
  const waypointName = name || `Waypoint ${waypointCounter++}`;
  const id = waypointId || `waypoint_${Date.now()}`;

  const existingNameWaypoint = waypointEntities.find(wp => wp.name === waypointName);
  if (existingNameWaypoint) {
    console.log("Waypoint with name already exists:", waypointName);
    return null;
  }

  const waypointEntity = viewer.entities.add({
    id: id,
    name: waypointName,
    position: Cesium.Cartesian3.fromDegrees(lon, lat),
    point: {
      pixelSize: 10,
      color: Cesium.Color.BLUE,
      outlineColor: Cesium.Color.WHITE,
      outlineWidth: 2,
      heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      disableDepthTestDistance: Number.POSITIVE_INFINITY
    },
    label: {
      text: waypointName,
      font: '14pt sans-serif',
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      outlineWidth: 2,
      verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
      pixelOffset: new Cesium.Cartesian2(0, -10),
      fillColor: Cesium.Color.WHITE,
      outlineColor: Cesium.Color.BLACK,
      showBackground: true,
      backgroundColor: new Cesium.Color(0.165, 0.165, 0.165, 0.7),
      heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
      disableDepthTestDistance: Number.POSITIVE_INFINITY
    }
  });

  if (waypointEntity._visualizers && waypointEntity._visualizers.length > 0) {
    waypointEntity._visualizers.forEach(visualizer => {
      if (visualizer && visualizer.visualizersByDisplayID) {
        for (const displayID in visualizer.visualizersByDisplayID) {
          if (visualizer.visualizersByDisplayID[displayID]) {
            visualizer.visualizersByDisplayID[displayID]._zIndex = 999;
          }
        }
      }
    });
  }

  waypointEntities.push(waypointEntity);
  return waypointEntity;
}

function showWaypointDialog(lat, lon) {
  Swal.fire({
    title: 'Add Waypoint',
    input: 'text',
    inputLabel: 'Waypoint Name',
    inputValue: `Waypoint ${waypointCounter}`,
    showCancelButton: true,
    confirmButtonText: 'Add',
    cancelButtonText: 'Cancel',
    inputValidator: (value) => {
      if (!value) return 'Please enter a name';

      const existingNameWaypoint = waypointEntities.find(wp => wp.name === value);
      if (existingNameWaypoint) {
        return 'A waypoint with this name already exists. Please choose a different name.';
      }
    }
  }).then((result) => {
    if (result.isConfirmed) {
      const name = result.value;
      const id = `waypoint_${Date.now()}`;

      const existingNameWaypoint = waypointEntities.find(wp => wp.name === name);
      if (existingNameWaypoint) {
        Swal.fire({
          title: 'Error',
          text: 'A waypoint with this name already exists. Please try again with a different name.',
          icon: 'error'
        });
        return;
      }

      const waypoint = addWaypoint(lat, lon, name, id);

      if (waypoint && window.bridge && window.bridge.waypointCreated) {
        window.bridge.waypointCreated(name, lat, lon, id);
      }
    }
  });
}