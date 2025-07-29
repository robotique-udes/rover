class Bridge {
    #viewInitialized = false;
    #lastHeading = 0;
    #currentPosition = {};

    constructor(viewer, roverEntity, waypointManager, cameraManager, initialPosition) 
    {
        this.viewer = viewer;
        this.rover = roverEntity;
        this.waypoints = waypointManager;
        this.camera = cameraManager;
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

            const self = this;

            qtBridge.clearPath.connect(function () 
            {
                self.waypoints.stopDynamicPathUpdates();
            });

            qtBridge.gpsCallback.connect(function (lat, lon, headingDeg) 
            {
                self.#gpsCallback(lat, lon, headingDeg);
            });

            qtBridge.sendGoal.connect(function (name, lat, lon) 
            {
                self.#setGoal(name, lat, lon);
            });

            qtBridge.calculatePath.connect(function (destLat, destLon, waypointId) 
            {
                self.waypoints.startDynamicPathUpdates(destLat, destLon, waypointId);
            });

            qtBridge.clearWaypoints.connect(function () 
            {
                self.waypoints.stopDynamicPathUpdates();
                self.waypoints.clearAllWaypoints();
            });

            qtBridge.deleteWaypoint.connect(function (waypointId) 
            {
                if (self.waypoints.activeWaypoint && self.waypoints.activeWaypoint.id === waypointId) 
                {
                    self.waypoints.stopDynamicPathUpdates();
                }
                self.waypoints.deleteWaypoint(waypointId);
            });
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

    #setGoal(name, lat, lon)
    {
        if (this.waypoints.isAddingWaypoint)
        {
            return;
        }

        // PR Étienne
        const id = `waypoint_${Date.now()}`;

        this.waypoints.addWaypoint(lat, lon, name, id);

        const wasTracking = this.camera.isCameraTracking;
        const wasTopDown = this.camera.isTopDownView;

        if (wasTracking) 
        {
            this.camera.toggleCameraTracking();
        }

        if (wasTopDown) {
            this.camera.toggleTopDownView();
        }

        this.viewer.camera.flyTo({
            destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1000.0),
            complete: () => {
                this.viewer.scene.requestRender();
            }
        });

        if (window.qtBridge && window.qtBridge.waypointCreated) 
        {
            window.qtBridge.waypointCreated(name, lat, lon, id);
        }
    }

}