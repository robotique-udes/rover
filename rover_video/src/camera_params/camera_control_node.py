#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import sys
import os

# Import the unified service message
from rover_msgs.srv import CameraParam

# Import the API file
from ipcamera_api import (
    CameraController, Scenes, ExposureModes, 
    WhiteBalanceModes, IRModes, FramerateValues,
    ShutterValues, AEGains
)

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        
        # Create callback group for services
        callback_group = ReentrantCallbackGroup()
        
        # No default camera parameters - will be passed in service calls
        self.camera_controller = None
        
        # Create a single unified service
        self.create_service(
            CameraParam, 'set_camera_param', 
            self.set_camera_param_callback, callback_group=callback_group
        )
        
        self.get_logger().info('Camera control service initialized')
    
    def set_camera_param_callback(self, request, response):
        # Get camera parameters from the request
        camera_ip = request.camera_ip if hasattr(request, 'camera_ip') else "192.168.1.18"
        camera_port = request.camera_port if hasattr(request, 'camera_port') else 8999
        camera_username = request.camera_username if hasattr(request, 'camera_username') else "admin"
        camera_password = request.camera_password if hasattr(request, 'camera_password') else "admin"
        camera_timeout = request.camera_timeout if hasattr(request, 'camera_timeout') else 5
        
        # Create a new controller for each request
        try:
            self.camera_controller = CameraController(
                self, camera_ip, camera_port, camera_username, camera_password, camera_timeout
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed to initialize camera controller: {e}"
            return response
        
        try:
            # Handle different parameter types
            if request.parameter_type == "int":
                result = self._set_int_parameter(request.parameter_name, request.int_value)
                
            elif request.parameter_type == "string":
                result = self._set_string_parameter(request.parameter_name, request.string_value)
                
            elif request.parameter_type == "enum":
                result = self._set_enum_parameter(request.parameter_name, request.int_value)
                
            elif request.parameter_type == "bool":
                result = self._set_bool_parameter(request.parameter_name, request.bool_value)
                
            elif request.parameter_type == "mirror":
                result = self._set_mirror_parameter(request.bool_value, request.bool_value2)
                
            else:
                response.success = False
                response.message = f"Unknown parameter type: {request.parameter_type}"
                return response
            
            if result:
                response.success = True
                response.message = f"Successfully set {request.parameter_name}"
            else:
                response.success = False
                response.message = f"Failed to set {request.parameter_name}"
                
        except Exception as e:
            response.success = False
            response.message = f"Error setting parameter: {str(e)}"
            self.get_logger().error(f"Error in service callback: {str(e)}")
        
        # Clean up controller
        self.camera_controller = None
            
        return response
    
    def _set_int_parameter(self, param_name, value):
        if param_name == "brightness":
            return self.camera_controller.setBrightness(value)
        elif param_name == "contrast":
            return self.camera_controller.setContrast(value)
        elif param_name == "saturation":
            return self.camera_controller.setSaturation(value)
        elif param_name == "sharpness":
            return self.camera_controller.setSharpness(value)
        elif param_name == "bitrate":
            return self.camera_controller.setBitrate(value)
        elif param_name == "wdr_level":
            return self.camera_controller.setWideDynamicRangeLevel(value)
        elif param_name == "ldc_level":
            return self.camera_controller.setLensDistortionCorrection(value)
        elif param_name == "antifog_level":
            return self.camera_controller.setAntiFog(value)
        else:
            self.get_logger().error(f"Unknown integer parameter: {param_name}")
            return False
    
    def _set_string_parameter(self, param_name, value):
        if param_name == "resolution":
            return self.camera_controller.setResolution(value)
        else:
            self.get_logger().error(f"Unknown string parameter: {param_name}")
            return False
    
    def _set_enum_parameter(self, param_name, value):
        try:
            if param_name == "framerate":
                framerate = FramerateValues(value)
                return self.camera_controller.setFrameRate(framerate)
            elif param_name == "scene":
                scene = Scenes(value)
                return self.camera_controller.setScene(scene)
            elif param_name == "exposure_mode":
                mode = ExposureModes(value)
                return self.camera_controller.setExposureMode(mode)
            elif param_name == "shutter_speed":
                speed = ShutterValues(value)
                return self.camera_controller.setShutterSpeed(speed)
            elif param_name == "manual_acg":
                gain = AEGains(value)
                return self.camera_controller.setManualACG(gain)
            elif param_name == "white_balance_mode":
                mode = WhiteBalanceModes(value)
                return self.camera_controller.setWhiteBalanceMode(mode)
            elif param_name == "ir_mode":
                mode = IRModes(value)
                return self.camera_controller.setIRMode(mode)
            else:
                self.get_logger().error(f"Unknown enum parameter: {param_name}")
                return False
        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Invalid enum value {value} for {param_name}: {e}")
            return False
    
    def _set_bool_parameter(self, param_name, value):
        if param_name == "wide_dynamic_range":
            if value:
                # When enabling, we set a default level of 128
                return self.camera_controller.setWideDynamicRangeLevel(128)
            else:
                return self.camera_controller.disableWideDynamicRange()
        elif param_name == "back_light":
            if value:
                return self.camera_controller.enableBackLight()
            else:
                return self.camera_controller.disableBackLight()
        elif param_name == "anti_false_color":
            if value:
                return self.camera_controller.enableAntiFalseColor()
            else:
                return self.camera_controller.disableAntiFalseColor()
        elif param_name == "digital_image_stabilizer":
            if value:
                return self.camera_controller.enableDigitalImageStabilizer()
            else:
                return self.camera_controller.disableDigitalImageStabilizer()
        elif param_name == "lens_shade_correction":
            if value:
                return self.camera_controller.enableLensShadeCorrection()
            else:
                return self.camera_controller.disableLensShadeCorrection()
        elif param_name == "lens_distortion_correction":
            if value:
                # When enabling, we set a default level of 128
                return self.camera_controller.setLensDistortionCorrection(128)
            else:
                return self.camera_controller.disableLensDistortionCorrection()
        elif param_name == "anti_fog":
            if value:
                # When enabling, we set a default level of 128
                return self.camera_controller.setAntiFog(128)
            else:
                return self.camera_controller.disableAntiFog()
        elif param_name == "ir":
            if value:
                # When enabling, we set auto mode by default
                return self.camera_controller.setIRMode(IRModes.AUTO)
            else:
                return self.camera_controller.disableIR()
        else:
            self.get_logger().error(f"Unknown boolean parameter: {param_name}")
            return False
    
    def _set_mirror_parameter(self, horizontal, vertical):
        success = True
        
        if horizontal:
            if not self.camera_controller.HorizontalMirror():
                success = False
        else:
            if not self.camera_controller.resetHorizontalMirror():
                success = False
            
        if vertical:
            if not self.camera_controller.VerticalMirror():
                success = False
        else:
            if not self.camera_controller.resetVerticalMirror():
                success = False
            
        return success

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

##################### EXAMPLE #######################
"""
ros2 service call /rover/video/set_camera_param rover_msgs/srv/CameraParam "{
  camera_ip: '192.168.1.18',
  camera_port: 8999,
  camera_username: 'admin',
  camera_password: 'admin',
  camera_timeout: 5,
  parameter_name: 'resolution',
  parameter_type: 'string',
  string_value: '1080p'
}"
"""