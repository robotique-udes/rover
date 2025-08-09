class PathManager
{
    #pathUpdateInterval = null;
    #pathEntities = [];
    #lastPosition = [];
    static WAYPOINT_PATH = "waypointPath";
    static POSITION_PATH = "positionPath";

    constructor(viewer, waypointManager)
    {
        this.viewer = viewer;
        this.waypointManager = waypointManager;
    }

    get pathEntities()
    {
        return this.#pathEntities;
    }

    clearWaypointPath()
    {
        if (Array.isArray(this.#pathEntities))
        {
            const idx = this.#pathEntities.findIndex(e => e.id === PathManager.WAYPOINT_PATH);
            if (idx !== -1)
            {
                this.viewer.entities.remove(this.#pathEntities[idx]);
                this.#pathEntities.splice(idx, 1);
            }
        }
    }

    startDynamicWaypointPathUpdates(destLat, destLon, waypointId) 
    {
        this.waypointManager.activeWaypoint = {
            latitude: destLat,
            longitude: destLon,
            id: waypointId
        };

        clearInterval(this.#pathUpdateInterval);
        this.#pathUpdateInterval = setInterval(() => {
            if (this.waypointManager.currentPosition && this.waypointManager.activeWaypoint) 
            {
                this.#drawLinePath(
                    PathManager.WAYPOINT_PATH,
                    this.waypointManager.currentPosition.latitude,
                    this.waypointManager.currentPosition.longitude,
                    this.waypointManager.activeWaypoint.latitude,
                    this.waypointManager.activeWaypoint.longitude
                );
            }
        }, 1000);

        this.#drawLinePath(
            PathManager.WAYPOINT_PATH,
            this.waypointManager.currentPosition.latitude,
            this.waypointManager.currentPosition.longitude,
            destLat,
            destLon
        );
    }

    stopDynamicWaypointPathUpdates() 
    {
        clearInterval(this.#pathUpdateInterval);
        this.#pathUpdateInterval = null;
        this.waypointManager.activeWaypoint = null;

        this.clearWaypointPath();
    }

    drawPathTaken(latitude_, longitude_)
    {
        this.#lastPosition.push([longitude_, latitude_]);
        const flatPositions = this.#lastPosition.flat();

        const idx = this.#pathEntities.findIndex(e => e.id === PathManager.POSITION_PATH);
        
        if (idx !== -1) 
        {
            this.viewer.entities.remove(this.#pathEntities[idx]);
            this.#pathEntities.splice(idx, 1);
        }

        const newEntity = this.viewer.entities.add({
            id: PathManager.POSITION_PATH,
            polyline: {
                positions: Cesium.Cartesian3.fromDegreesArray(flatPositions),
                width: 3,
                material: new Cesium.PolylineOutlineMaterialProperty({
                    color: Cesium.Color.CYAN,
                    outlineWidth: 1,
                    outlineColor: Cesium.Color.BLACK
                }),
                clampToGround: true
            }
        });
        this.#pathEntities.push(newEntity);
    }

    #calculateHaversineDistance(lat1, lon1, lat2, lon2) 
    {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;

        const a = Math.sin(dLat / 2) ** 2 +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) ** 2;

        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }

    #drawLinePath(name, startLat, startLon, endLat, endLon) 
    {
        const idx = this.#pathEntities.findIndex(e => e.id === name);
        if (idx !== -1) 
        {
            this.viewer.entities.remove(this.#pathEntities[idx]);
            this.#pathEntities.splice(idx, 1);
        }

        let newEntity = this.viewer.entities.add({
            id: name,
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

        this.#pathEntities.push(newEntity);

        const distance = this.#calculateHaversineDistance(startLat, startLon, endLat, endLon);
        if (window.qtBridge) 
        {
            window.qtBridge.pathDistanceCalculated(distance);
        }
        return distance;
    }
}