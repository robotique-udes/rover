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
    const initialPosition = { latitude: 45.377755, longitude: -71.924652 }
    window.viewer = new Cesium.Viewer("cesiumContainer", {
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

    setupControlButtons();

    window.camera = new Camera(viewer);
    window.waypointManager = new Waypoint(viewer);
    window.bridge = new Bridge(viewer, null, waypointManager, camera, initialPosition);

    window.roverEntity = viewer.entities.add({
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
            const headingRad = Cesium.Math.toRadians(window.bridge.lastHeading || 0);
            return Cesium.Transforms.headingPitchRollQuaternion(
                Cesium.Cartesian3.fromDegrees(window.bridge.currentPosition.longitude, window.bridge.currentPosition.latitude),
                new Cesium.HeadingPitchRoll(headingRad, 0, 0)
            );
        }, false)
    });

    window.bridge.rover = roverEntity;

    setupEventHandlers();
}

function setupControlButtons() 
{
    const trackingButton = document.getElementById('trackingButton');
    const topDownButton = document.getElementById('topDownButton');
    const northFacingButton = document.getElementById('northFacingButton');

    if (trackingButton)
    {
        trackingButton.addEventListener('click', function () {
            camera.toggleCameraTracking();
        });
    }

    if (topDownButton)
    {
        topDownButton.addEventListener('click', function () {
            camera.toggleTopDownView();
        });
    }

    if (northFacingButton)
    {
        northFacingButton.addEventListener('click', function () {
            camera.toggleNorthFacing();
        });
    }
}

function setupEventHandlers() 
{
    viewer.screenSpaceEventHandler.setInputAction(onDoubleClick, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    viewer.screenSpaceEventHandler.setInputAction(onLeftDown, Cesium.ScreenSpaceEventType.LEFT_DOWN);
    viewer.screenSpaceEventHandler.setInputAction(onWheel, Cesium.ScreenSpaceEventType.WHEEL);
}

function onDoubleClick(event)
{
    if (camera.isCameraTracking)
    {
        camera.toggleCameraTracking();
    }

    try {
        const scene = viewer.scene;

        let cartesian = pickPositionFromTerrain(scene, event)
            || pickPositionFromDrillPick(scene, event)
            || pickPositionFromRay(scene, event)
            || pickPositionFromEllipsoid(scene, event);

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
}

function onLeftDown(event)
{
    if (camera.isCameraTracking) 
    {
        camera.toggleCameraTracking();
    }
}

function onWheel(event)
{
    if (camera.isCameraTracking) 
    {
        camera.toggleCameraTracking();
    }
}

function pickPositionFromTerrain(scene, event) 
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

function pickPositionFromDrillPick(scene, event) 
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

function pickPositionFromRay(scene, event) 
{
    const ray = viewer.camera.getPickRay(event.position);
    if (Cesium.defined(ray)) 
    {
        return viewer.scene.globe.pick(ray, viewer.scene);
    }
    return undefined;
}

function pickPositionFromEllipsoid(scene, event) 
{
    return viewer.camera.pickEllipsoid(
        event.position,
        viewer.scene.globe.ellipsoid
    );
}