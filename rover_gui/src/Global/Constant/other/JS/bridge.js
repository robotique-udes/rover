function setupBridgeConnection() {
  new QWebChannel(qt.webChannelTransport, function (channel) {
    const bridge = channel.objects.bridge;
    window.bridge = bridge;

    let viewInitialized = false;

    bridge.clearPath.connect(function () {
      stopDynamicPathUpdates();
    });

    bridge.gpsCallback.connect(function (lat, lon, headingDeg) {
      currentPosition.latitude = lat;
      currentPosition.longitude = lon;
      lastHeading = headingDeg;

      roverEntity.position = Cesium.Cartesian3.fromDegrees(lon, lat);

      if (!viewInitialized) {
        try {
          viewer.camera.setView({
            destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1000.0),
            orientation: {
              heading: 0.0,
              pitch: -90.0,
              roll: 0.0
            }
          });
          viewInitialized = true;
        } catch (error) {
          console.error("Error initializing view:", error);
        }
      }

      if (cameraTracking) {
        try {
          updateCameraPosition();
        } catch (error) {
          console.error("Error updating camera position:", error);
          if (cameraTracking) {
            toggleCameraTracking();
          }
        }
      }

      if (isTopDownView) {
        try {
          setTopDownView();
        } catch (error) {
          console.error("Error updating top-down view:", error);
        }
      }
    });

    bridge.sendGoal.connect(function (name, lat, lon) {
      if (isAddingWaypoint) return;

      isAddingWaypoint = true;
      const id = `waypoint_${Date.now()}`;

      const existingNameWaypoint = waypointEntities.find(wp =>
        wp.name === name
      );

      if (existingNameWaypoint) {
        Swal.fire({
          title: 'Duplicate Waypoint',
          text: `A waypoint named "${name}" already exists. Please use a different name.`,
          icon: 'warning',
          confirmButtonText: 'OK'
        });
        isAddingWaypoint = false;
        return;
      }

      const existingLocationWaypoint = waypointEntities.find(wp => {
        const wpPosition = wp.position.getValue(Cesium.JulianDate.now());
        const wpCartographic = Cesium.Cartographic.fromCartesian(wpPosition);
        const wpLat = Cesium.Math.toDegrees(wpCartographic.latitude);
        const wpLon = Cesium.Math.toDegrees(wpCartographic.longitude);

        const epsilon = 0.00001;
        return Math.abs(wpLat - lat) < epsilon && Math.abs(wpLon - lon) < epsilon;
      });

      if (existingLocationWaypoint) {
        Swal.fire({
          title: 'Duplicate Location',
          text: `A waypoint already exists at this location. Please choose a different location.`,
          icon: 'warning',
          confirmButtonText: 'OK'
        });
        isAddingWaypoint = false;
        return;
      }

      addWaypoint(lat, lon, name, id);

      const wasTracking = cameraTracking;
      const wasTopDown = isTopDownView;

      if (cameraTracking) {
        toggleCameraTracking();
      }

      if (isTopDownView) {
        toggleTopDownView();
      }

      viewer.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1000.0),
        complete: function () {
          viewer.scene.requestRender();
          isAddingWaypoint = false;

          if (wasTracking) {
            toggleCameraTracking();
          }

          if (wasTopDown) {
            toggleTopDownView();
          }
        }
      });

      if (window.bridge && window.bridge.waypointCreated) {
        window.bridge.waypointCreated(name, lat, lon, id);
      }
    });

    bridge.calculatePath.connect(function (destLat, destLon, waypointId) {
      startDynamicPathUpdates(destLat, destLon, waypointId);
    });

    bridge.clearWaypoints.connect(function () {
      stopDynamicPathUpdates();
      clearAllWaypoints();
    });

    bridge.deleteWaypoint.connect(function (waypointId) {
      if (activeWaypoint && activeWaypoint.id === waypointId) {
        stopDynamicPathUpdates();
      }
      deleteWaypoint(waypointId);
    });

    if (bridge.setCameraTracking) {
      bridge.setCameraTracking.connect(function (enabled) {
        if (cameraTracking !== enabled) {
          toggleCameraTracking();
        }
      });
    }

    if (bridge.setTopDownView) {
      bridge.setTopDownView.connect(function (enabled) {
        if (isTopDownView !== enabled) {
          toggleTopDownView();
        }
      });
    }

    if (bridge.jsReady) {
      bridge.jsReady();
    }
  });
}