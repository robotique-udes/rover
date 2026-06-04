class Bridge 
{
    #viewInitialized = false;
    #lastHeading = 0;
    #currentPosition = {};

    constructor(viewer, roverEntity, waypointManager, cameraManager, initialPosition, pathManager) 
    {
        this.viewer = viewer;
        this.rover = roverEntity;
        this.waypoints = waypointManager;
        this.camera = cameraManager;
        this.pathManager = pathManager;
        this.#currentPosition = initialPosition || { latitude: 0, longitude: 0 };
        this.#setupBridgeConnection();
    }

    get lastHeading()
    {
        return this.#lastHeading;
    }

    get currentPosition()
    {
        return this.#currentPosition;
    }

    #setupBridgeConnection()
    {
        new QWebChannel(qt.webChannelTransport, (channel) => 
        {
            const qtBridge = channel.objects.bridge;
            window.qtBridge = qtBridge;

            qtBridge.clearPath.connect(() =>
            {
                this.pathManager.stopDynamicWaypointPathUpdates();
            });

            qtBridge.gpsCallback.connect((lat, lon, headingDeg) => 
            {
                this.#gpsCallback(lat, lon, headingDeg);
            });

            qtBridge.sendGoal.connect((name, lat, lon, id, flyTo) => 
            {
                this.#setGoal(name, lat, lon, id, flyTo);
            });

            qtBridge.calculatePath.connect((destLat, destLon, waypointId) => 
            {
                this.pathManager.startDynamicWaypointPathUpdates(destLat, destLon, waypointId);
            });

            qtBridge.clearWaypoints.connect(() => 
            {
                Swal.fire({
                    title: 'Waypoints cleared',
                    text: `All waypoints have been removed.`,
                    icon: 'success',
                    confirmButtonText: 'OK'
                });

                this.waypoints.clearAllWaypoints();
                this.pathManager.stopDynamicWaypointPathUpdates();
                this.pathManager.clearWaypointPath();
            });

            qtBridge.deleteWaypoint.connect((waypointId) =>
            {
                if (this.waypoints.activeWaypoint && this.waypoints.activeWaypoint.id === waypointId) 
                {
                    this.pathManager.stopDynamicWaypointPathUpdates();
                }
                this.waypoints.deleteWaypoint(waypointId);
            });

            qtBridge.waypointIsVisible.connect((waypointId, visibility) =>
            {
                this.waypoints.waypointVisibility(waypointId, visibility);
            });

            qtBridge.updatePathTaken.connect((latitude_, longitude_) =>
            {
                this.pathManager.drawPathTaken(latitude_, longitude_);
            });

            qtBridge.loadFullPath.connect((points_) =>
            {
               this.pathManager.drawFullPath(points_) 
            });

            if (window.qtBridge && window.qtBridge.onJsBridgeReady) 
            {
                window.qtBridge.onJsBridgeReady();
            }
        });

        
    }

    #gpsCallback(lat, lon, headingDeg)
    {
        this.#currentPosition.latitude = lat;
        this.#currentPosition.longitude = lon;
        this.#lastHeading = headingDeg;
        this.camera.currentPosition = this.#currentPosition;
        this.waypoints.currentPosition = this.#currentPosition;

        this.rover.position = Cesium.Cartesian3.fromDegrees(lon, lat);

        if (!this.#viewInitialized) 
        {
            try {
                this.viewer.camera.setView({
                    destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1000.0),
                    orientation: {
                    heading: 0.0,
                    pitch: -90.0,
                    roll: 0.0
                    }
                });
                this.#viewInitialized = true;
            } catch (error) {
                console.error("Error initializing view:", error);
            }
        }
    }

    #setGoal(name, lat, lon, id, flyTo)
    {
        if (this.waypoints.isAddingWaypoint)
        {
            return;
        }

        const waypoint = this.waypoints.addWaypoint(lat, lon, name, id);
        if (!waypoint) 
        {
            return;
        }

        const wasTracking = this.camera.isCameraTracking;
        const wasTopDown = this.camera.isTopDownView;

        if (wasTracking) 
        {
            this.camera.toggleCameraTracking();
        }

        if (wasTopDown) {
            this.camera.toggleTopDownView();
        }

        if (flyTo)
        {
            this.viewer.camera.flyTo({
                destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1000.0),
                complete: () => {
                    this.viewer.scene.requestRender();
                }
            });
        }

        if (window.qtBridge && window.qtBridge.waypointCreated) 
        {
            window.qtBridge.waypointCreated(name, lat, lon, id);
        }
    }

}