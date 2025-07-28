async function checkConnectivity() 
{
    const connectionError = document.getElementById('connectionError');

    if (!navigator.onLine)
    {
        connectionError.style.display = 'block';
        return false;
    } 
    
    try {
        await fetch('https://cesium.com/downloads/cesiumjs/releases/1.114/Build/Cesium/Cesium.js', {
            method: 'HEAD',
            mode: 'no-cors',
            cache: 'no-store'
        });
        connectionError.style.display = 'none';
        return true;
    } catch {
        connectionError.style.display = 'block';
        return false;
    }
}

function setupCesiumMap() 
{
    viewer = new Cesium.Viewer("cesiumContainer", {
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

    viewer._cesiumWidget._creditContainer.style.display = "none";

    roverEntity = viewer.entities.add({
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
        orientation: new Cesium.CallbackProperty(function () {
            const headingRad = Cesium.Math.toRadians(lastHeading || 0);
            return Cesium.Transforms.headingPitchRollQuaternion(
                Cesium.Cartesian3.fromDegrees(bridge.currentPosition().longitude, bridge.currentPosition().latitude),
                new Cesium.HeadingPitchRoll(headingRad, 0, 0)
            );
        }, false)
    });

    window.camera = new Camera(viewer);
    window.waypointManager = new Waypoint(viewer);
    window.bridge = new Bridge(viewer, roverEntity);

    setupEventHandlers();
    setupControlButtons();
}

function setupControlButtons() 
{
    const trackingButton = document.getElementById('trackingButton');
    const topDownButton = document.getElementById('topDownButton');
    const northFacingButton = document.getElementById('northFacingButton');

    trackingButton.addEventListener('click', function () {
        camera.toggleCameraTracking();
    });

    topDownButton.addEventListener('click', function () {
        camera.toggleTopDownView();
    });

    northFacingButton.addEventListener('click', function () {
        camera.toggleNorthFacing();
    });
    }

function setupEventHandlers() 
{
    viewer.screenSpaceEventHandler.setInputAction(function (click) {
        if (camera.cameraTracking())
        {
            camera.toggleCameraTracking();
        }

        try {
            let cartesian;
            const scene = viewer.scene;

            if (scene.terrainProvider.ready) 
            {
                cartesian = scene.pickPosition(click.position);

                if (cartesian) 
                {
                    const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
                    if (cartographic.height > 10000 || cartographic.height < -1000) 
                    {
                        cartesian = undefined;
                    }
                }
            }

            if (!Cesium.defined(cartesian)) 
            {
                const drillPickResult = scene.drillPick(click.position);
                if (drillPickResult.length > 0) 
                {
                    for (let i = 0; i < drillPickResult.length; i++) 
                    {
                        if (Cesium.defined(drillPickResult[i].primitive) && Cesium.defined(drillPickResult[i].primitive.position)) 
                        {
                            cartesian = drillPickResult[i].primitive.position;
                            break;
                        }
                    }
                }
            }

            if (!Cesium.defined(cartesian)) 
            {
                const ray = viewer.camera.getPickRay(click.position);
                if (Cesium.defined(ray)) 
                {
                    cartesian = viewer.scene.globe.pick(ray, viewer.scene);
                }
            }

            if (!Cesium.defined(cartesian)) 
            {
                cartesian = viewer.camera.pickEllipsoid(
                    click.position,
                    viewer.scene.globe.ellipsoid
                );
            }

            if (Cesium.defined(cartesian)) 
            {
                const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
                const lat = Cesium.Math.toDegrees(cartographic.latitude);
                const lon = Cesium.Math.toDegrees(cartographic.longitude);

                setTimeout(() => {
                    waypointManager.showWaypointDialog(lat, lon);
                }, 50);
            }
        } catch (error) {
            // #TODO: Wasnt implemented in the original code
        }
    }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);

    viewer.screenSpaceEventHandler.setInputAction(function () {
        if (camera.isCameraTracking()) 
        {
            camera.toggleCameraTracking();
        }
    }, Cesium.ScreenSpaceEventType.LEFT_DOWN);

    viewer.screenSpaceEventHandler.setInputAction(function () {
        if (camera.cameraTracking()) 
        {
            camera.toggleCameraTracking();
        }
    }, Cesium.ScreenSpaceEventType.WHEEL);
}