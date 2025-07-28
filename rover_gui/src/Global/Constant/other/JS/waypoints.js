class Waypoint {
    #waypointCounter = 1;
    #activeWaypoint = null;
    #isAddingWaypoint = false;
    #pathEntity = null;
    #waypointEntities = [];
    #pathUpdateInterval = null;
    /* Maybe use a global currentPosition ---> to think about */
    #currentPosition = { latitude: 45.377755, longitude: -71.924652 };


    constructor(viewer) 
    {
        this.viewer = viewer;
    }

    set currentPosition(position)
    {
        this.#currentPosition = position;
    }

    get getIsAddingWaypoint()
    {
        return this.#isAddingWaypoint;
    }

    // Temporary
    set setIsAddingWaypoint(value)
    {
        this.#isAddingWaypoint = value;
    }

    get waypointEntities()
    {
        return this.#waypointEntities;
    }

    get activeWaypoint()
    {
        return this.#activeWaypoint;
    }

    clearAllWaypoints() 
    {
        this.#waypointEntities.forEach(waypoint => {
            this.viewer.entities.remove(waypoint);
        });
        this.#waypointEntities = [];
        this.#waypointCounter = 1;

        if (this.#pathEntity) 
        {
            this.viewer.entities.remove(this.#pathEntity);
            this.#pathEntity = null;
        }
    }

    deleteWaypoint(waypointId) 
    {
        const entity = this.viewer.entities.getById(waypointId);
        if (entity) 
        {
            this.viewer.entities.remove(entity);
            this.#waypointEntities = this.#waypointEntities.filter(wp => wp.id !== waypointId);
            return true;
        }
        return false;
    }

    #drawPath(startLat, startLon, endLat, endLon) 
    {
        if (this.#pathEntity) 
        {
            this.viewer.entities.remove(this.#pathEntity);
        }

        this.#pathEntity = this.viewer.entities.add({
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

        const distance = this.#calculateHaversineDistance(startLat, startLon, endLat, endLon);
        if (window.bridge) 
        {
            window.bridge.pathDistanceCalculated(distance);
        }
        return distance;
    }

    startDynamicPathUpdates(destLat, destLon, waypointId) 
    {
        this.#activeWaypoint = {
            latitude: destLat,
            longitude: destLon,
            id: waypointId
        };

        clearInterval(this.#pathUpdateInterval);
        this.#pathUpdateInterval = setInterval(() => {
            if (this.#currentPosition && this.#activeWaypoint) 
            {
                this.#drawPath(
                    this.#currentPosition.latitude,
                    this.#currentPosition.longitude,
                    this.#activeWaypoint.latitude,
                    this.#activeWaypoint.longitude
                );
            }
        }, 1000);

        this.#drawPath(
            this.#currentPosition.latitude,
            this.#currentPosition.longitude,
            destLat,
            destLon
        );
    }

    stopDynamicPathUpdates() 
    {
        clearInterval(this.#pathUpdateInterval);
        this.#pathUpdateInterval = null;
        this.#activeWaypoint = null;

        if (this.#pathEntity) 
        {
            this.viewer.entities.remove(this.#pathEntity);
            this.#pathEntity = null;
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

    addWaypoint(lat, lon, name, waypointId) 
    {
        const existingName = this.#checkName(name);
        const existingLocation = this.#checkLocation(lat, lon);

        if (existingName || existingLocation)
        {
            return;
        }

        const waypointName = name || `Waypoint ${this.#waypointCounter++}`;
        const id = waypointId || `waypoint_${Date.now()}`;

        const waypointEntity = this.viewer.entities.add({
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
            if (visualizer && visualizer.visualizersByDisplayID) 
            {
                for (const displayID in visualizer.visualizersByDisplayID)
                {
                    if (visualizer.visualizersByDisplayID[displayID]) 
                    {
                        visualizer.visualizersByDisplayID[displayID]._zIndex = 999;
                    }
                }
            }
            });
        }

        this.#waypointEntities.push(waypointEntity);
        return waypointEntity;
    }

    showWaypointDialog(lat, lon) 
    {
        Swal.fire({
            title: 'Add Waypoint',
            input: 'text',
            inputLabel: 'Waypoint Name',
            inputValue: `Waypoint ${this.#waypointCounter}`,
            showCancelButton: true,
            confirmButtonText: 'Add',
            cancelButtonText: 'Cancel',
            inputValidator: (value) => {
                if (!value)
                {
                    return 'Please enter a name';
                } 

                const existingNameWaypoint = this.#waypointEntities.find(wp => wp.name === value);
                if (existingNameWaypoint) 
                {
                    return 'A waypoint with this name already exists. Please choose a different name.';
                }
            }
        }).then((result) => {
            if (result.isConfirmed) 
            {
                const name = result.value;
                const id = `waypoint_${Date.now()}`;

                const existingNameWaypoint = this.#waypointEntities.find(wp => wp.name === name);
                if (existingNameWaypoint) 
                {
                    Swal.fire({
                    title: 'Error',
                    text: 'A waypoint with this name already exists. Please try again with a different name.',
                    icon: 'error'
                    });
                    return;
                }

                const waypoint = this.addWaypoint(lat, lon, name, id);

                if (waypoint && window.bridge && window.bridge.waypointCreated) 
                {
                    window.bridge.waypointCreated(name, lat, lon, id);
                }
            }
        });
    }

    #checkName(name)
    {
        const existingName = waypointEntities.find(wp =>
            wp.name === name
        );

        if (existingName) 
        {
            Swal.fire({
                title: 'Duplicate Waypoint',
                text: `A waypoint named "${name}" already exists. Please use a different name.`,
                icon: 'warning',
                confirmButtonText: 'OK'
            });
            waypointManager.setIsAddingWaypoint(false);
            return existingName;
        }
        return !existingName
    }

    #checkLocation(lat, lon)
    {
        const existingLocation = waypointEntities.find(wp => {
            const wpPosition = wp.position.getValue(Cesium.JulianDate.now());
            const wpCartographic = Cesium.Cartographic.fromCartesian(wpPosition);
            const wpLat = Cesium.Math.toDegrees(wpCartographic.latitude);
            const wpLon = Cesium.Math.toDegrees(wpCartographic.longitude);

            const epsilon = 0.00001;
            return Math.abs(wpLat - lat) < epsilon && Math.abs(wpLon - lon) < epsilon;
        });

        if (existingLocation) 
        {
            Swal.fire({
            title: 'Duplicate Location',
            text: `A waypoint already exists at this location. Please choose a different location.`,
            icon: 'warning',
            confirmButtonText: 'OK'
            });
            waypointManager.setIsAddingWaypoint(false);
            return existingLocation;
        }

        return !existingLocation;
    }
}