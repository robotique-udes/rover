class PathManager
{
    #pathUpdateInterval = null;
    #pathEntities = [];
    #currentPath = [];
    #oldPath = [];
    static WAYPOINT_PATH = "waypointPath";
    static POSITION_PATH = "currentPath";
    static OLD_POSITION_PATH = "oldPath";

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

    drawPathTaken(latitude_, longitude_, name_)
    {
        let newEntity = null;
        if (name_ === PathManager.OLD_POSITION_PATH)
        {
            this.#oldPath.push([longitude_, latitude_]);
            const flatPositions = this.#oldPath.flat();
    
            const idx = this.#pathEntities.findIndex(e => e.id === PathManager.OLD_POSITION_PATH);
            
            if (idx !== -1) 
            {
                this.viewer.entities.remove(this.#pathEntities[idx]);
                this.#pathEntities.splice(idx, 1);
            }
    
            newEntity = this.viewer.entities.add({
                id: name_,
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
        }
        else if (name_ === PathManager.POSITION_PATH)
        {
            this.#currentPath.push([longitude_, latitude_]);
            const flatPositions = this.#currentPath.flat();
    
            const idx = this.#pathEntities.findIndex(e => e.id === PathManager.POSITION_PATH);
            
            if (idx !== -1) 
            {
                this.viewer.entities.remove(this.#pathEntities[idx]);
                this.#pathEntities.splice(idx, 1);
            }
    
            newEntity = this.viewer.entities.add({
                id: name_,
                polyline: {
                    positions: Cesium.Cartesian3.fromDegreesArray(flatPositions),
                    width: 3,
                    material: new Cesium.PolylineOutlineMaterialProperty({
                        color: Cesium.Color.RED,
                        outlineWidth: 1,
                        outlineColor: Cesium.Color.BLACK
                    }),
                    clampToGround: true
                }
            });
        }

        if(newEntity)
        {
            this.#pathEntities.push(newEntity);
        }
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
        const heading = this.#calculateHeading(startLat, startLon, endLat, endLon);
        if (window.qtBridge) 
        {
            window.qtBridge.pathDistanceCalculated(distance, heading);
        }
        return distance;
    }

    #calculateHeading(lat1, lon1, lat2, lon2)
    {
        const dLon = (lon2 - lon1) * Math.PI / 180;
        lat1 = lat1 * Math.PI / 180;
        lat2 = lat2 * Math.PI / 180;

        const y = Math.sin(dLon) * Math.cos(lat2);
        const x = Math.cos(lat1) * Math.sin(lat2) -
                  Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
        const heading = Math.atan2(y, x);

        return (heading * 180 / Math.PI + 360) % 360; // Convert to degrees and normalize
    }
}