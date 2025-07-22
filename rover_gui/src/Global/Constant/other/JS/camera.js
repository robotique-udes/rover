class Camera 
{
    #_isFacingNorth = false;
    #_isTopDownView = false;
    #_isAdjustingCamera = false;
    #_northFacingTimeout = null;
    #_lastCameraPosition = null;
    #_cameraTracking = false;
    /* Remember about currentPosition and how to declare it later on */

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
            if (!this.#_cameraTracking) 
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

                if (this.#_lastCameraPosition && !this.#_cameraTracking) 
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

            const targetLongitude = this.#_cameraTracking ? currentPosition.longitude : this.viewer.camera.positionCartographic.longitude * 180.0 / Math.PI;
            const targetLatitude = this.#_cameraTracking ? currentPosition.latitude : this.viewer.camera.positionCartographic.latitude * 180.0 / Math.PI;

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
                                            currentPosition.longitude,
                                            currentPosition.latitude,
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
        this.#_cameraTracking = !this.#_cameraTracking;
        const trackingButton = document.getElementById('trackingButton');
        const trackingText = document.getElementById('trackingText');

        if (this.#_cameraTracking)
        {
            trackingButton.classList.add('active');
            trackingText.textContent = 'Tracking On';

            try {
                this.#updateCameraPosition();
            } catch (error) {
                console.error("Error enabling tracking:", error);
                this.#_cameraTracking = false;
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
        if (!this.#_cameraTracking || !this.viewer || !this.viewer.camera)
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
                    currentPosition.longitude,
                    currentPosition.latitude,
                    cameraHeight
                ),
                orientation: orientation
            });
        } catch (error) {
            console.error("Failed to update camera position:", error);
            try {
                this.viewer.camera.flyTo({
                    destination: Cesium.Cartesian3.fromDegrees(
                        currentPosition.longitude,
                        currentPosition.latitude,
                        1000.0
                    )
                });
            } catch (flyError) {
                console.error("Even fallback camera update failed:", flyError);
                this.#_cameraTracking = false;
                document.getElementById('trackingButton').classList.remove('active');
                document.getElementById('trackingText').textContent = 'Track Position';
            }
        }
    }
};

// function toggleNorthFacing() {
//   isFacingNorth = !isFacingNorth;
//   const northFacingButton = document.getElementById('northFacingButton');
//   const northFacingText = document.getElementById('northFacingText');

//   if (isFacingNorth) {
//     northFacingButton.classList.add('active');
//     northFacingText.textContent = 'Facing North';
//     setNorthFacing();

//     setTimeout(() => {
//       if (isFacingNorth) {
//         viewer.camera.changed.addEventListener(stillFacingNorth);
//       }
//     }, 100);
//   }
//   else {
//     northFacingButton.classList.remove('active');
//     northFacingText.textContent = 'Face North'
//     viewer.camera.changed.removeEventListener(stillFacingNorth);

//     if (northFacingTimeout) {
//       clearTimeout(northFacingTimeout);
//       northFacingTimeout = null;
//     }
//   }
// }

// function setNorthFacing() {
//   if (isFacingNorth) {
//     const desiredHeading = 0.0;
//     const headingThreshold = 0.01;

//     const needHeadingAdjustment = Math.abs(viewer.camera.heading - desiredHeading) > headingThreshold

//     if (needHeadingAdjustment) {
//       isAdjustingCamera = true;
//       viewer.camera.setView(
//         {
//           orientation:
//           {
//             heading: desiredHeading,
//             pitch: viewer.camera.pitch,
//             roll: 0.0
//           }
//         });
//       isAdjustingCamera = false;
//     }
//   }
// }

// function stillFacingNorth() {
//   if (!isFacingNorth || isAdjustingCamera) {
//     return;
//   }

//   if (northFacingTimeout) {
//     clearTimeout(northFacingTimeout);
//   }

//   northFacingTimeout = setTimeout(() => {
//     const currentHeading = viewer.camera.heading;
//     const desiredHeading = 0.0;
//     const headingThreshold = 0.05;
//     const notOnNorth = Math.abs(currentHeading - desiredHeading) > headingThreshold;

//     if (notOnNorth) {
//       toggleNorthFacing();
//     }
//     northFacingTimeout = null;
//   }, 200);
// }

// function toggleTopDownView() {
//   isTopDownView = !isTopDownView;
//   const topDownButton = document.getElementById('topDownButton');
//   const topDownText = document.getElementById('topDownText');

//   if (isTopDownView) {
//     if (!cameraTracking) {
//       lastCameraPosition = {
//         position: viewer.camera.position.clone(),
//         heading: viewer.camera.heading,
//         pitch: viewer.camera.pitch,
//         roll: viewer.camera.roll
//       };
//     }

//     topDownButton.classList.add('active');
//     topDownText.textContent = 'Exit Top-Down';

//     try {
//       setTopDownView();

//       if (!isFacingNorth) {
//         toggleNorthFacing();
//       }

//       if (viewer && viewer.scene && viewer.scene.screenSpaceCameraController) {
//         viewer.scene.screenSpaceCameraController.enableTilt = false;
//       }

//       viewer.camera.changed.addEventListener(maintainTopDownPerspective);
//     } catch (error) {
//       console.error("Error enabling top-down view:", error);
//       isTopDownView = false;
//       topDownButton.classList.remove('active');
//       topDownText.textContent = 'Top-Down View';
//     }
//   } else {
//     topDownButton.classList.remove('active');
//     topDownText.textContent = 'Top-Down View';

//     if (isFacingNorth) {
//       toggleNorthFacing();
//     }

//     try {
//       if (viewer && viewer.scene && viewer.scene.screenSpaceCameraController) {
//         viewer.scene.screenSpaceCameraController.enableTilt = true;
//       }

//       viewer.camera.changed.removeEventListener(maintainTopDownPerspective);

//       if (lastCameraPosition && !cameraTracking) {
//         viewer.camera.setView({
//           destination: lastCameraPosition.position,
//           orientation: {
//             heading: lastCameraPosition.heading,
//             pitch: lastCameraPosition.pitch,
//             roll: lastCameraPosition.roll
//           }
//         });
//       }
//     } catch (error) {
//       console.error("Error disabling top-down view:", error);
//     }
//   }

//   try {
//     if (window.bridge && window.bridge.topDownViewChanged) {
//       window.bridge.topDownViewChanged(isTopDownView); // #TODO: Not implemented in C++
//     }
//   } catch (error) {
//     console.error("Error notifying bridge about top-down state:", error);
//   }
// }

// function setTopDownView() {
//   if (!isTopDownView || !viewer || !viewer.camera) return;

//   try {
//     let cameraHeight = 1000.0;
//     try {
//       if (viewer.camera.positionCartographic) {
//         cameraHeight = Math.max(viewer.camera.positionCartographic.height, 1000.0);
//       }
//     } catch (e) {
//       console.warn("Could not get camera height, using default:", e);
//     }

//     const targetLongitude = cameraTracking ? currentPosition.longitude : viewer.camera.positionCartographic.longitude * 180.0 / Math.PI;
//     const targetLatitude = cameraTracking ? currentPosition.latitude : viewer.camera.positionCartographic.latitude * 180.0 / Math.PI;

//     isAdjustingCamera = true;
//     viewer.camera.setView({
//       destination: Cesium.Cartesian3.fromDegrees(
//         targetLongitude,
//         targetLatitude,
//         cameraHeight
//       ),
//       orientation: {
//         heading: 0.0,
//         pitch: -Math.PI / 2,
//         roll: 0.0
//       }
//     });
//     isAdjustingCamera = false;
//   } catch (error) {
//     console.error("Failed to update camera to top-down view:", error);
//     try {
//       isAdjustingCamera = true;
//       viewer.camera.flyTo({
//         destination: Cesium.Cartesian3.fromDegrees(
//           currentPosition.longitude,
//           currentPosition.latitude,
//           1000.0
//         ),
//         orientation: {
//           heading: viewer.camera.heading,
//           pitch: -Math.PI / 2,
//           roll: 0.0
//         }
//       });
//       isAdjustingCamera = false;
//     } catch (flyError) {
//       console.error("Even fallback camera update failed:", flyError);
//       isTopDownView = false;
//       document.getElementById('topDownButton').classList.remove('active');
//       document.getElementById('topDownText').textContent = 'Top-Down View';
//     }
//   }
// }

// function maintainTopDownPerspective() {
//   if (!isTopDownView || isAdjustingCamera) return;

//   const currentPitch = viewer.camera.pitch;
//   const desiredPitch = -Math.PI / 2;
//   const pitchThreshold = 0.01;

//   const needPitchAdjustment = Math.abs(currentPitch - desiredPitch) > pitchThreshold;

//   if (needPitchAdjustment) {
//     isAdjustingCamera = true;
//     viewer.camera.setView({
//       orientation: {
//         heading: viewer.camera.heading,
//         pitch: desiredPitch,
//         roll: 0.0
//       }
//     });
//     isAdjustingCamera = false;
//   }
// }

// function toggleCameraTracking() {
//   cameraTracking = !cameraTracking;
//   const trackingButton = document.getElementById('trackingButton');
//   const trackingText = document.getElementById('trackingText');

//   if (cameraTracking) {
//     trackingButton.classList.add('active');
//     trackingText.textContent = 'Tracking On';

//     try {
//       updateCameraPosition();

//     } catch (error) {
//       console.error("Error enabling tracking:", error);
//       cameraTracking = false;
//       trackingButton.classList.remove('active');
//       trackingText.textContent = 'Track Position';
//     }
//   } else {
//     trackingButton.classList.remove('active');
//     trackingText.textContent = 'Track Position';

//   }

//   try {
//     if (window.bridge && window.bridge.cameraTrackingChanged) {
//       window.bridge.cameraTrackingChanged(cameraTracking); // #TODO: Not implemented in C++
//     }
//   } catch (error) {
//     console.error("Error notifying bridge about tracking state:", error);
//   }
// }

// function updateCameraPosition() {
//   if (!cameraTracking || !viewer || !viewer.camera) return;

//   try {
//     let cameraHeight = 1000.0;
//     try {
//       if (viewer.camera.positionCartographic) {
//         cameraHeight = viewer.camera.positionCartographic.height;
//       }
//     } catch (e) {
//       console.warn("Could not get camera height, using default:", e);
//     }

//     const orientation = {
//       heading: viewer.camera.heading,
//       pitch: viewer.camera.pitch,
//       roll: 0.0
//     };

//     viewer.camera.setView({
//       destination: Cesium.Cartesian3.fromDegrees(
//         currentPosition.longitude,
//         currentPosition.latitude,
//         cameraHeight
//       ),
//       orientation: orientation
//     });
//   } catch (error) {
//     console.error("Failed to update camera position:", error);
//     try {
//       viewer.camera.flyTo({
//         destination: Cesium.Cartesian3.fromDegrees(
//           currentPosition.longitude,
//           currentPosition.latitude,
//           1000.0
//         )
//       });
//     } catch (flyError) {
//       console.error("Even fallback camera update failed:", flyError);
//       cameraTracking = false;
//       document.getElementById('trackingButton').classList.remove('active');
//       document.getElementById('trackingText').textContent = 'Track Position';
//     }
//   }
// }