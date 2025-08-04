class Camera 
{
    static DEFAULT_CAMERA_HEIGHT = 1000.0;

    #isFacingNorth = false;
    #isTopDownView = false;
    #isAdjustingCamera = false;
    #northFacingTimeout = null;
    #lastCameraPosition = null;
    #isCameraTracking = false;
    #currentPosition = {};

    #boundStillFacingNorth = this.#stillFacingNorth.bind(this);
    #boundMaintainTopDownPerspective = this.#maintainTopDownPerspective.bind(this);

    constructor(viewer) 
    {
        this.viewer = viewer;
        this.trackingButton = document.getElementById('trackingButton');
        this.trackingText = document.getElementById('trackingText');
        this.topDownButton = document.getElementById('topDownButton');
        this.topDownText = document.getElementById('topDownText');
        this.northFacingButton = document.getElementById('northFacingButton');
        this.northFacingText = document.getElementById('northFacingText');
    }

    get isFacingNorth() 
    {
        return this.#isFacingNorth;
    }

    get isTopDownView()
    {
        return this.#isTopDownView;
    }

    get isAdjustingCamera()
    {
        return this.#isAdjustingCamera;
    }

    get isCameraTracking()
    {
        return this.#isCameraTracking;
    }

    set currentPosition(position)
    {
        this.#currentPosition = position;
        if (this.#isCameraTracking)
        {
            this.#updateCameraPosition();
        }
    }

    toggleNorthFacing()
    {
        this.#isFacingNorth = !this.#isFacingNorth;

        if (this.#isFacingNorth) 
        {
            this.#changeButtonStyle(this.northFacingButton, this.northFacingText, true, 'Face North', 'Facing North');
            this.#setNorthFacing();

            setTimeout(() => {
                if (this.#isFacingNorth) 
                {
                    this.viewer.camera.changed.addEventListener(this.#boundStillFacingNorth);
                }
            }, 100);
        }
        else 
        {
            this.#changeButtonStyle(this.northFacingButton, this.northFacingText, false, 'Face North', 'Facing North');
            this.viewer.camera.changed.removeEventListener(this.#boundStillFacingNorth);

            if (this.#northFacingTimeout) 
            {
                clearTimeout(this.#northFacingTimeout);
                this.#northFacingTimeout = null;
            }
        }
    }

    #setNorthFacing()
    {
        if (this.#isFacingNorth)
        {
            const desiredHeading = 0.0;
            const headingThreshold = 0.01;

            const needHeadingAdjustment = Math.abs(this.viewer.camera.heading - desiredHeading) > headingThreshold

            if (needHeadingAdjustment)
            {
                this.#isAdjustingCamera = true;
                this.viewer.camera.setView({ orientation:
                                            {
                                                heading: desiredHeading,
                                                pitch: this.viewer.camera.pitch,
                                                roll: 0.0
                                            }
                });
                this.#isAdjustingCamera = false;
            }
        }
    }

    #stillFacingNorth()
    {
        if (!this.#isFacingNorth || this.#isAdjustingCamera) 
        {
            return;
        }

        if (this.#northFacingTimeout) 
        {
            clearTimeout(this.#northFacingTimeout);
        }

        this.#northFacingTimeout = setTimeout(() => {
            const currentHeading = this.viewer.camera.heading;
            const desiredHeading = 0.0;
            const headingThreshold = 0.05;
            const notOnNorth = Math.abs(currentHeading - desiredHeading) > headingThreshold;

            if (notOnNorth) 
            {
                this.toggleNorthFacing();
            }
            this.#northFacingTimeout = null;
        }, 200);
    }

    toggleTopDownView()
    {
        this.#isTopDownView = !this.#isTopDownView;

        if (this.#isTopDownView) {
            if (!this.#isCameraTracking) 
            {
                this.#lastCameraPosition = {
                    position: this.viewer.camera.position.clone(),
                    heading: this.viewer.camera.heading,
                    pitch: this.viewer.camera.pitch,
                    roll: this.viewer.camera.roll
                };
            }

            this.#changeButtonStyle(this.topDownButton, this.topDownText, true, 'Top-Down View', 'Exit Top-Down');

            try {
                this.#setTopDownView();

                if (!this.#isFacingNorth) 
                {
                    this.toggleNorthFacing();
                }

                if (this.viewer && this.viewer.scene && this.viewer.scene.screenSpaceCameraController) {
                    this.viewer.scene.screenSpaceCameraController.enableTilt = false;
                }

                this.viewer.camera.changed.addEventListener(this.#boundMaintainTopDownPerspective);
            } catch (error) {
                console.error("Error enabling top-down view:", error);
                this.#isTopDownView = false;
                this.#changeButtonStyle(this.topDownButton, this.topDownText, false, 'Top-Down View', 'Exit Top-Down');
            }
        } 
        else 
        {
            this.#changeButtonStyle(this.topDownButton, this.topDownText, false, 'Top-Down View', 'Exit Top-Down');

            if (this.#isFacingNorth) 
            {
                this.toggleNorthFacing();
            }

            try {
                if (this.viewer && this.viewer.scene && this.viewer.scene.screenSpaceCameraController) {
                    this.viewer.scene.screenSpaceCameraController.enableTilt = true;
                }

                this.viewer.camera.changed.removeEventListener(this.#boundMaintainTopDownPerspective);

                if (this.#lastCameraPosition && !this.#isCameraTracking) 
                {
                    this.viewer.camera.setView({
                        destination: this.#lastCameraPosition.position,
                        orientation: 
                        {
                            heading: this.#lastCameraPosition.heading,
                            pitch: this.#lastCameraPosition.pitch,
                            roll: this.#lastCameraPosition.roll
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
        if (!this.#isTopDownView) 
        {
            return;
        }

        try {
            let cameraHeight = Camera.DEFAULT_CAMERA_HEIGHT;
            try {
                if (this.viewer.camera.positionCartographic) 
                {
                    cameraHeight = Math.max(this.viewer.camera.positionCartographic.height, Camera.DEFAULT_CAMERA_HEIGHT);
                }
            } catch (e) {
                console.warn("Could not get camera height, using default:", e);
            }

            const targetLongitude = this.#isCameraTracking ? this.#currentPosition.longitude : this.viewer.camera.positionCartographic.longitude * 180.0 / Math.PI;
            const targetLatitude = this.#isCameraTracking ? this.#currentPosition.latitude : this.viewer.camera.positionCartographic.latitude * 180.0 / Math.PI;

            this.#isAdjustingCamera = true;
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
            this.#isAdjustingCamera = false;
        } catch (error) {
            console.error("Failed to update camera to top-down view:", error);
            try {
                this.#isAdjustingCamera = true;
                this.viewer.camera.flyTo({
                                        destination: Cesium.Cartesian3.fromDegrees(
                                            this.#currentPosition.longitude,
                                            this.#currentPosition.latitude,
                                            Camera.DEFAULT_CAMERA_HEIGHT
                                        ),
                    orientation: 
                    {
                        heading: this.viewer.camera.heading,
                        pitch: -Math.PI / 2,
                        roll: 0.0
                    }
                });
                this.#isAdjustingCamera = false;
            } catch (flyError) {
                console.error("Even fallback camera update failed:", flyError);
                this.#isTopDownView = false;
                this.#changeButtonStyle(this.topDownButton, this.topDownText, false, 'Top-Down View', 'Exit Top-Down');
            }
        }
    }

    #maintainTopDownPerspective()
    {
        if (!this.#isTopDownView || this.#isAdjustingCamera)
        {
            return;
        }

        const currentPitch = this.viewer.camera.pitch;
        const desiredPitch = -Math.PI / 2;
        const pitchThreshold = 0.01;

        const needPitchAdjustment = Math.abs(currentPitch - desiredPitch) > pitchThreshold;

        if (needPitchAdjustment) {
            this.#isAdjustingCamera = true;
            this.viewer.camera.setView({
                orientation:
                {
                    heading: this.viewer.camera.heading,
                    pitch: desiredPitch,
                    roll: 0.0
                }
            });
            this.#isAdjustingCamera = false;
        }
    }

    toggleCameraTracking()
    {
        this.#isCameraTracking = !this.#isCameraTracking;

        if (this.#isCameraTracking)
        {
            this.#changeButtonStyle(this.trackingButton, this.trackingText, true, 'Track Position', 'Tracking On');

            try {
                this.#updateCameraPosition();
            } catch (error) {
                console.error("Error enabling tracking:", error);
                this.#isCameraTracking = false;
                this.#changeButtonStyle(this.trackingButton, this.trackingText, false, 'Track Position', 'Tracking On');
            }
        }
        else
        {
            this.#changeButtonStyle(this.trackingButton, this.trackingText, false, 'Track Position', 'Tracking On');

        }

        // #TODO: Not implemented in C++
        // try {
        //     if (window.bridge && window.bridge.cameraTrackingChanged) {
        //         window.bridge.cameraTrackingChanged(cameraTracking); 
        //     }
        // } catch (error) {
        //     console.error("Error notifying bridge about tracking state:", error);
        // }
    }

    #updateCameraPosition()
    {
        if (!this.#isCameraTracking || !this.viewer || !this.viewer.camera)
        {
            return;
        }

        try {
            let cameraHeight = Camera.DEFAULT_CAMERA_HEIGHT;
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
                    this.#currentPosition.longitude,
                    this.#currentPosition.latitude,
                    cameraHeight
                ),
                orientation: orientation
            });
        } catch (error) {
            console.error("Failed to update camera position:", error);
            try {
                this.viewer.camera.flyTo({
                    destination: Cesium.Cartesian3.fromDegrees(
                        this.#currentPosition.longitude,
                        this.#currentPosition.latitude,
                        Camera.DEFAULT_CAMERA_HEIGHT
                    )
                });
            } catch (flyError) {
                console.error("Even fallback camera update failed:", flyError);
                this.#isCameraTracking = false;
                this.#changeButtonStyle(this.trackingButton, this.trackingText, false, 'Track Position', 'Tracking On');
            }
        }
    }

    #changeButtonStyle(button, textElement, isActive, inactiveText, activeText) 
    {
        if (isActive) 
        {
            button.classList.add('active');
            textElement.textContent = activeText;
        } 
        else 
        {
            button.classList.remove('active');
            textElement.textContent = inactiveText;
        }
    }
}