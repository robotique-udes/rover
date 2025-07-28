class Bridge {
    #viewInitialized = false;
    #lastHeading = 0;
    #currentPosition = { latitude: 45.377755, longitude: -71.924652 };

    constructor(viewer, roverEntity) 
    {
        this.viewer = viewer;
        this.rover = roverEntity;
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
            const bridge = channel.objects.bridge;
            window.bridge = bridge;

            const self = this;

            bridge.clearPath.connect(function () 
            {
                waypointManager.stopDynamicPathUpdates();
            });

            bridge.gpsCallback.connect(function (lat, lon, headingDeg) 
            {
                self.#gpsCallback(lat, lon, headingDeg);
            });

            bridge.sendGoal.connect(function (name, lat, lon) 
            {
                self.#setGoal(name, lat, lon);
            });

            bridge.calculatePath.connect(function (destLat, destLon, waypointId) 
            {
                waypointManager.startDynamicPathUpdates(destLat, destLon, waypointId);
            });

            bridge.clearWaypoints.connect(function () 
            {
                waypointManager.stopDynamicPathUpdates();
                waypointManager.clearAllWaypoints();
            });

            bridge.deleteWaypoint.connect(function (waypointId) 
            {
                if (activeWaypoint && activeWaypoint.id === waypointId) 
                {
                    waypointManager.stopDynamicPathUpdates();
                }
                waypointManager.deleteWaypoint(waypointId);
            });
        });
    }

    #gpsCallback(lat, lon, headingDeg)
    {
        this.#currentPosition.latitude = lat;
        this.#currentPosition.longitude = lon;
        this.#lastHeading = headingDeg;

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
        if (waypoint.getIsAddingWaypoint())
        {
            return;
        }

        waypointManager.setIsAddingWaypoint(true);
        // PR Étienne
        const id = `waypoint_${Date.now()}`;

        waypointManager.addWaypoint(lat, lon, name, id);

        const wasTracking = camera.getIsCameraTracking();
        const wasTopDown = camera.isTopDownView();

        if (camera.isCameraTracking()) 
        {
            camera.toggleCameraTracking();
        }

        if (camera.isTopDownView())
        {
            camera.toggleTopDownView();
        }

        viewer.camera.flyTo({
            destination: Cesium.Cartesian3.fromDegrees(lon, lat, 1000.0),
            complete: function () {
                viewer.scene.requestRender();
                waypointManager.setIsAddingWaypoint(false);

                if (wasTracking) 
                {
                    camera.toggleCameraTracking();
                }

                if (wasTopDown) {
                    camera.toggleTopDownView();
                }
            }
        });

        if (window.bridge && window.bridge.waypointCreated) 
        {
            window.bridge.waypointCreated(name, lat, lon, id);
        }
    }

}