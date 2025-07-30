class MapManager
{
    constructor(containerId) 
    {
        this.viewer = new Cesium.Viewer(containerId, {
            terrain: Cesium.Terrain.fromWorldTerrain({
            requestWaterMask: true,
            requestVertexNormals: true,
            }),
            animation: false,
            baseLayerPicker: false,
            fullscreenButton: false,
            vrButton: false,
            geocoder: false,
            homeButton: false,
            infoBox: false,
            sceneModePicker: false,
            selectionIndicator: false,
            timeline: false,
            navigationHelpButton: false,
            navigationInstructionsInitiallyVisible: false,
            creditsDisplay: false,
            shouldAnimate: true,
        });

        this.#setupControlButtons();

        this.camera = new Camera(this.viewer);
        this.waypointManager = new Waypoint(this.viewer);
        this.bridge = new Bridge(this.viewer, null, this.waypointManager, this.camera, { latitude: 45.377755, longitude: -71.924652 });
        this.roverEntity = this.viewer.entities.add({
                name: "Live Position Arrow",
                position: Cesium.Cartesian3.fromDegrees(0.0, 0.0, 0),
                model: {
                    uri: 'qrc:/model/direction_arrow.glb',
                    scale: 0.2,
                    minimumPixelSize: 30,
                    maximumScale: 60,
                    heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
                    rotation: 0,
                    color: Cesium.Color.RED,
                },
            orientation: new Cesium.CallbackProperty(() => {
                const headingRad = Cesium.Math.toRadians(this.bridge.lastHeading || 0);
                return Cesium.Transforms.headingPitchRollQuaternion(
                    Cesium.Cartesian3.fromDegrees(this.bridge.currentPosition.longitude, this.bridge.currentPosition.latitude),
                    new Cesium.HeadingPitchRoll(headingRad, 0, 0)
                );
            }, false)
        });

        this.bridge.rover = this.roverEntity;

        this.#setupEventHandlers();
    }

    #setupControlButtons()
    {
        const trackingButton = document.getElementById('trackingButton');
        const topDownButton = document.getElementById('topDownButton');
        const northFacingButton = document.getElementById('northFacingButton');

        if (trackingButton)
        {
            trackingButton.addEventListener('click', () => {
                this.camera.toggleCameraTracking();
            });
        }

        if (topDownButton)
        {
            topDownButton.addEventListener('click', () => {
                this.camera.toggleTopDownView();
            });
        }

        if (northFacingButton)
        {
            northFacingButton.addEventListener('click', () => {
                this.camera.toggleNorthFacing();
            });
        }
    }
    
    #setupEventHandlers() 
    {
        this.viewer.screenSpaceEventHandler.setInputAction(this.#onDoubleClick.bind(this), Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
        this.viewer.screenSpaceEventHandler.setInputAction(this.#onLeftDown.bind(this), Cesium.ScreenSpaceEventType.LEFT_DOWN);
        this.viewer.screenSpaceEventHandler.setInputAction(this.#onWheel.bind(this), Cesium.ScreenSpaceEventType.WHEEL);
    }

    #onDoubleClick(event)
    {
        if (this.camera.isCameraTracking)
        {
            this.camera.toggleCameraTracking();
        }

        try {
            const scene = this.viewer.scene;

            let cartesian = this.#pickPositionFromTerrain(scene, event)
                || this.#pickPositionFromDrillPick(scene, event)
                || this.#pickPositionFromRay(scene, event)
                || this.#pickPositionFromEllipsoid(scene, event);

            if (Cesium.defined(cartesian)) 
            {
                const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
                const lat = Cesium.Math.toDegrees(cartographic.latitude);
                const lon = Cesium.Math.toDegrees(cartographic.longitude);

                setTimeout(() => {
                    this.waypointManager.showWaypointDialog(lat, lon);
                }, 50);
            }
        } catch (error) {
            // #TODO: Wasnt implemented in the original code
        }
    }

    #onLeftDown(event)
    {
        if (this.camera.isCameraTracking) 
        {
            this.camera.toggleCameraTracking();
        }
    }

    #onWheel(event)
    {
        if (this.camera.isCameraTracking) 
        {
            this.camera.toggleCameraTracking();
        }
    }
    
    #pickPositionFromTerrain(scene, event)
    {
        if (!scene.terrainProvider.ready)
        {
            return undefined;
        }

        let cartesian = scene.pickPosition(event.position);
        if (cartesian) 
        {
            const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
            if (cartographic.height > 10000 || cartographic.height < -1000) 
            {
                return undefined;
            }
        }
        return cartesian;
    }

    #pickPositionFromDrillPick(scene, event)
    {
        const drillPickResult = scene.drillPick(event.position);
        if (drillPickResult.length > 0) 
        {
            for (let i = 0; i < drillPickResult.length; i++) 
            {
                if (Cesium.defined(drillPickResult[i].primitive) && Cesium.defined(drillPickResult[i].primitive.position)) 
                {
                    return drillPickResult[i].primitive.position;
                }
            }
        }
        return undefined;
    }

    #pickPositionFromRay(scene, event)
    {
        const ray = this.viewer.camera.getPickRay(event.position);
        if (Cesium.defined(ray)) 
        {
            return this.viewer.scene.globe.pick(ray, this.viewer.scene);
        }
        return undefined;
    }

    #pickPositionFromEllipsoid(scene, event)
    {
        return this.viewer.camera.pickEllipsoid(
            event.position,
            this.viewer.scene.globe.ellipsoid
        );
    }
}