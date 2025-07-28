class Camera 
{
    #_isFacingNorth = false;
    #_isTopDownView = false;
    #_isAdjustingCamera = false;
    #_northFacingTimeout = null;
    #_lastCameraPosition = null;
    #_isCameraTracking = false;
    #_currentPosition = { latitude: 45.377755, longitude: -71.924652 };

    #_boundStillFacingNorth = this.#stillFacingNorth.bind(this);
    #_boundMaintainTopDownPerspective = this.#maintainTopDownPerspective.bind(this);

    constructor(viewer) 
    {
        this.viewer = viewer;
    }

    get isFacingNorth() 
    {
        return this.#_isFacingNorth;
    }

    get isTopDownView()
    {
        return this.#_isTopDownView;
    }

    get isAdjustingCamera()
    {
        return this.#_isAdjustingCamera;
    }

    get isCameraTracking()
    {
        return this.#_isCameraTracking;
    }

    set currentPosition(position)
    {
        this.#_currentPosition = position;
    }

    toggleNorthFacing()
    {
        this.#_isFacingNorth = !this.#_isFacingNorth;
        const northFacingButton = document.getElementById('northFacingButton');
        const northFacingText = document.getElementById('northFacingText');

        if (this.#_isFacingNorth) 
        {
            northFacingButton.classList.add('active');
            northFacingText.textContent = 'Facing North';
            this.#setNorthFacing();

            setTimeout(() => {
                if (this.#_isFacingNorth) 
                {
                    this.viewer.camera.changed.addEventListener(this.#_boundStillFacingNorth);
                }
            }, 100);
        }
        else 
        {
            northFacingButton.classList.remove('active');
            northFacingText.textContent = 'Face North'
            this.viewer.camera.changed.removeEventListener(this.#_boundStillFacingNorth);

            if (this.#_northFacingTimeout) 
            {
                clearTimeout(this.#_northFacingTimeout);
                this.#_northFacingTimeout = null;
            }
        }
    }

    #setNorthFacing()
    {
        if (this.#_isFacingNorth)
        {
            const desiredHeading = 0.0;
            const headingThreshold = 0.01;

            const needHeadingAdjustment = Math.abs(this.viewer.camera.heading - desiredHeading) > headingThreshold

            if (needHeadingAdjustment)
            {
                this.#_isAdjustingCamera = true;
                this.viewer.camera.setView({ orientation:
                                            {
                                                heading: desiredHeading,
                                                pitch: this.viewer.camera.pitch,
                                                roll: 0.0
                                            }
                });
                this.#_isAdjustingCamera = false;
            }
        }
    }

    #stillFacingNorth()
    {
        if (!this.#_isFacingNorth || this.#_isAdjustingCamera) 
        {
            return;
        }

        if (this.#_northFacingTimeout) 
        {
            clearTimeout(this.#_northFacingTimeout);
        }

        this.#_northFacingTimeout = setTimeout(() => {
            const currentHeading = this.viewer.camera.heading;
            const desiredHeading = 0.0;
            const headingThreshold = 0.05;
            const notOnNorth = Math.abs(currentHeading - desiredHeading) > headingThreshold;

            if (notOnNorth) 
            {
                this.toggleNorthFacing();
            }
            this.#_northFacingTimeout = null;
        }, 200);
    }

    toggleTopDownView()
    {
        this.#_isTopDownView = !this.#_isTopDownView;
        const topDownButton = document.getElementById('topDownButton');
        const topDownText = document.getElementById('topDownText');

        if (this.#_isTopDownView) {
            if (!this.#_isCameraTracking) 
            {
                this.#_lastCameraPosition = {
                    position: this.viewer.camera.position.clone(),
                    heading: this.viewer.camera.heading,
                    pitch: this.viewer.camera.pitch,
                    roll: this.viewer.camera.roll
                };
            }

            topDownButton.classList.add('active');
            topDownText.textContent = 'Exit Top-Down';

            try {
                this.#setTopDownView();

                if (!this.#_isFacingNorth) 
                {
                    this.toggleNorthFacing();
                }

                if (this.viewer && this.viewer.scene && this.viewer.scene.screenSpaceCameraController) {
                    this.viewer.scene.screenSpaceCameraController.enableTilt = false;
                }

                this.viewer.camera.changed.addEventListener(this.#_boundMaintainTopDownPerspective);
            } catch (error) {
                console.error("Error enabling top-down view:", error);
                this.#_isTopDownView = false;
                topDownButton.classList.remove('active');
                topDownText.textContent = 'Top-Down View';
            }
        } 
        else 
        {
            topDownButton.classList.remove('active');
            topDownText.textContent = 'Top-Down View';

            if (this.#_isFacingNorth) 
            {
                this.toggleNorthFacing();
            }

            try {
                if (this.viewer && this.viewer.scene && this.viewer.scene.screenSpaceCameraController) {
                    this.viewer.scene.screenSpaceCameraController.enableTilt = true;
                }

                this.viewer.camera.changed.removeEventListener(this.#_boundMaintainTopDownPerspective);

                if (this.#_lastCameraPosition && !this.#_isCameraTracking) 
                {
                    this.viewer.camera.setView({
                        destination: this.#_lastCameraPosition.position,
                        orientation: 
                        {
                            heading: this.#_lastCameraPosition.heading,
                            pitch: this.#_lastCameraPosition.pitch,
                            roll: this.#_lastCameraPosition.roll
                        }
                    });
                }
            } catch (error) {
                console.error("Error disabling top-down view:", error);
            }
        }
    }

    #setTopDownView()
    {
        if (!this.#_isTopDownView || !this.viewer || !this.viewer.camera) 
        {
            return;
        }

        try {
            let cameraHeight = 1000.0;
            try {
                if (this.viewer.camera.positionCartographic) 
                {
                    cameraHeight = Math.max(this.viewer.camera.positionCartographic.height, 1000.0);
                }
            } catch (e) {
                console.warn("Could not get camera height, using default:", e);
            }

            const targetLongitude = this.#_isCameraTracking ? this.#_currentPosition.longitude : this.viewer.camera.positionCartographic.longitude * 180.0 / Math.PI;
            const targetLatitude = this.#_isCameraTracking ? this.#_currentPosition.latitude : this.viewer.camera.positionCartographic.latitude * 180.0 / Math.PI;

            this.#_isAdjustingCamera = true;
            this.viewer.camera.setView({
                destination: Cesium.Cartesian3.fromDegrees(
                    targetLongitude,
                    targetLatitude,
                    cameraHeight
                ),
                orientation: 
                {
                    heading: 0.0,
                    pitch: -Math.PI / 2,
                    roll: 0.0
                }
            });
            this.#_isAdjustingCamera = false;
        } catch (error) {
            console.error("Failed to update camera to top-down view:", error);
            try {
                this.#_isAdjustingCamera = true;
                this.viewer.camera.flyTo({
                                        destination: Cesium.Cartesian3.fromDegrees(
                                            this.#_currentPosition.longitude,
                                            this.#_currentPosition.latitude,
                                            1000.0
                                        ),
                    orientation: 
                    {
                        heading: this.viewer.camera.heading,
                        pitch: -Math.PI / 2,
                        roll: 0.0
                    }
                });
                this.#_isAdjustingCamera = false;
            } catch (flyError) {
                console.error("Even fallback camera update failed:", flyError);
                this.#_isTopDownView = false;
                document.getElementById('topDownButton').classList.remove('active');
                document.getElementById('topDownText').textContent = 'Top-Down View';
            }
        }
    }

    #maintainTopDownPerspective()
    {
        if (!this.#_isTopDownView || this.#_isAdjustingCamera)
        {
            return;
        }

        const currentPitch = this.viewer.camera.pitch;
        const desiredPitch = -Math.PI / 2;
        const pitchThreshold = 0.01;

        const needPitchAdjustment = Math.abs(currentPitch - desiredPitch) > pitchThreshold;

        if (needPitchAdjustment) {
            this.#_isAdjustingCamera = true;
            this.viewer.camera.setView({
                orientation:
                {
                    heading: this.viewer.camera.heading,
                    pitch: desiredPitch,
                    roll: 0.0
                }
            });
            this.#_isAdjustingCamera = false;
        }
    }

    toggleCameraTracking()
    {
        this.#_isCameraTracking = !this.#_isCameraTracking;
        const trackingButton = document.getElementById('trackingButton');
        const trackingText = document.getElementById('trackingText');

        if (this.#_isCameraTracking)
        {
            trackingButton.classList.add('active');
            trackingText.textContent = 'Tracking On';

            try {
                this.#updateCameraPosition();
            } catch (error) {
                console.error("Error enabling tracking:", error);
                this.#_isCameraTracking = false;
                trackingButton.classList.remove('active');
                trackingText.textContent = 'Track Position';
            }
        }
        else
        {
            trackingButton.classList.remove('active');
            trackingText.textContent = 'Track Position';

        }

        // try {
        //     if (window.bridge && window.bridge.cameraTrackingChanged) {
        //         window.bridge.cameraTrackingChanged(cameraTracking); // #TODO: Not implemented in C++
        //     }
        // } catch (error) {
        //     console.error("Error notifying bridge about tracking state:", error);
        // }
    }

    #updateCameraPosition()
    {
        if (!this.#_isCameraTracking || !this.viewer || !this.viewer.camera)
        {
            return;
        }

        try {
            let cameraHeight = 1000.0;
            try {
                if (this.viewer.camera.positionCartographic)
                {
                    cameraHeight = this.viewer.camera.positionCartographic.height;
                }
            } catch (e) {
                console.warn("Could not get camera height, using default:", e);
            }

            const orientation = {
                heading: this.viewer.camera.heading,
                pitch: this.viewer.camera.pitch,
                roll: 0.0
            };

            this.viewer.camera.setView({
                destination: Cesium.Cartesian3.fromDegrees(
                    this.#_currentPosition.longitude,
                    this.#_currentPosition.latitude,
                    cameraHeight
                ),
                orientation: orientation
            });
        } catch (error) {
            console.error("Failed to update camera position:", error);
            try {
                this.viewer.camera.flyTo({
                    destination: Cesium.Cartesian3.fromDegrees(
                        this.#_currentPosition.longitude,
                        this.#_currentPosition.latitude,
                        1000.0
                    )
                });
            } catch (flyError) {
                console.error("Even fallback camera update failed:", flyError);
                this.#_isCameraTracking = false;
                document.getElementById('trackingButton').classList.remove('active');
                document.getElementById('trackingText').textContent = 'Track Position';
            }
        }
    }
}