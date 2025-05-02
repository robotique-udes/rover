import requests
import signal
import string
from onvif import ONVIFCamera
from enum import Enum
import time

class TimeoutException(Exception):
    pass

class Scenes(Enum):
    OUTDOOR = 0
    INDOOR = 2

class ExposureModes(Enum):
    SCENE = 0
    MANUAL = 1
    SHUTTER = 2

class WhiteBalanceModes(Enum):
    AUTO = 0
    MANUAL = 1
    INDOOR = 8
    OUTDOOR = 9
    SUNLIGHT = 2

class IRModes(Enum):
    AUTO = 4
    DAY = 3
    NIGHT = 2

class FramerateValues(Enum):
    _3 = 3
    _5 = 5
    _10 = 10
    _15 = 15
    _20 = 20
    _25 = 25
    _30 = 30

class ShutterValues(Enum):
    _1_8000 = 0
    _1_6000 = 1
    _1_4000 = 2
    _1_2000 = 3
    _1_1000 = 4
    _1_500 = 5
    _1_250 = 6
    _1_200 = 7
    _1_150 = 8
    _1_100 = 9
    _1_50 = 10
    _1_25 = 11
    _1_20 = 12
    _1_15 = 13
    _1_10 = 14
    _1_8 = 15
    _1_5 = 16
    _1_3 = 17
    _1_2 = 18
    _1 = 19

class AEGains(Enum):
    _1X = 0
    _2X = 1
    _4X = 2
    _8X = 3
    _16X = 4
    _32X = 5
    _64X = 6

# Create a wrapper class for dictionary-style loggers
class DictLogger:
    def __init__(self, log_dict):
        self.log_dict = log_dict
    
    def info(self, msg):
        if 'info' in self.log_dict and callable(self.log_dict['info']):
            self.log_dict['info'](msg)
    
    def error(self, msg):
        if 'error' in self.log_dict and callable(self.log_dict['error']):
            self.log_dict['error'](msg)

class CameraController:
    ONVIF_PARAMS = {
        'Brightness': 'imaging',
        'Contrast': 'imaging',
        'ColorSaturation': 'imaging', 
        'Sharpness': 'imaging',
        'Resolution': 'video',
        'FrameRateLimit': 'video',
        'BitrateLimit': 'video'
    }

    HTTP_PARAMS = {
        'WideDynamicRange': ('WDR', 1),
        'WideDynamicRangeLevel': ('WDRLEVEL', 16),
        'BackLight': ('BLC', 1),
        'MirrorHorizontal': ('MHR', 1),
        'MirrorVertical': ('MVR', 1),
        'AntiFalseColor': ('AFC', 9),
        'DigitalImageStabilizer': ('DIS', 12),
        'LensShadeCorrection': ('LSC', 11),
        'LensDistortionCorrection': ('LDC', 15),
        'LensDistortionCorrectionLevel': ('LDCLevel', 19),
        'AntiFog': ('AFG', 10),
        'AntiFogLevel': ('AFGLevel', 20),
        'SceneSelect': ('scene_Select', 2),
        'ExposureMode': ('AEMode_Select', 2),
        'ShutterSpeed': ('ShutterSpeedText', 2),
        'ManualACG': ('AEGainText', 2),
        'IRenable': ('IRenable', 0),
        'IRmode': ('IRmode', 0),
        'WhiteBalanceModeSelect': ('WBMode_Select', 3)
    }

    RESOLUTION_MAP = {
        '480p': (640, 480),
        'w480p': (720, 480),
        '576p': (720, 576),
        '720p': (1280, 720),
        '960p': (1280, 960),
        '1080p': (1920, 1080),
        '1536p': (2048, 1536)
    }

    # Static cache for controller instances to avoid multiple connections
    _controllers = {}

    def __init__(self, node, ip: str, port: int, username: str, password: str, timeout: int = 5):
        # Check if we already have a controller for this IP
        controller_key = f"{ip}:{port}:{username}:{password}"
        existing_controller = CameraController._controllers.get(controller_key)
        
        if existing_controller is not None:
            # Copy instance variables from existing controller
            self.__dict__.update(existing_controller.__dict__)
            return
            
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self.onvif_connected = False
        self.connection_retries = 0
        self.max_retries = 3
        # Track which parameters have been explicitly set
        self.set_params = set()
        self.verbose_logging = False  # Set to False to reduce logging
        
        # Handle various node/logger formats
        if hasattr(node, 'get_logger') and callable(node.get_logger):
            # It's a ROS2 node with get_logger method
            self.logger = node.get_logger()
        elif isinstance(node, dict) and 'get_logger' in node and callable(node['get_logger']):
            # It's a dictionary with get_logger function that returns a dict of log functions
            logger_dict = node['get_logger']()
            if isinstance(logger_dict, dict):
                # Create a wrapper for the dictionary-style logger
                self.logger = DictLogger(logger_dict)
            else:
                # Direct use if it's already a logger object
                self.logger = logger_dict
        else:
            # Fallback to a minimal print-based logger
            class PrintLogger:
                def info(self, msg): print(f"INFO: {msg}")
                def error(self, msg): print(f"ERROR: {msg}")
            
            self.logger = PrintLogger()
            if self.verbose_logging:
                print(f"Warning: Using minimal logger. Node type: {type(node)}")

        # Try to connect via ONVIF with retries
        self._try_onvif_connection(timeout)
        
        # Store in the static cache
        CameraController._controllers[controller_key] = self

    def _try_onvif_connection(self, timeout):
        while self.connection_retries < self.max_retries:
            signal.signal(signal.SIGALRM, self.timeout_handler)
            signal.alarm(timeout)

            try:
                if self.verbose_logging:
                    self.logger.info(f"Attempting ONVIF connection to {self.ip} (attempt {self.connection_retries + 1}/{self.max_retries})")
                self.camera = ONVIFCamera(self.ip, self.port, self.username, self.password)

                self.media_service = self.camera.create_media_service()
                profiles = self.media_service.GetProfiles()
                if not profiles:
                    raise Exception("No media profiles found on the camera")
                self.profile = profiles[0]

                self.imaging_service = self.camera.create_imaging_service()
                self.source_token = self.profile.VideoSourceConfiguration.SourceToken
                
                self.onvif_connected = True
                self.logger.info(f"ONVIF connection successful to {self.ip}")
                break

            except TimeoutException:
                self.logger.error(f"Camera initialization timed out (attempt {self.connection_retries + 1}/{self.max_retries})")
            except Exception as e:
                self.logger.error(f"Failed to initialize camera via ONVIF (attempt {self.connection_retries + 1}/{self.max_retries}): {str(e)}")
            finally:
                signal.alarm(0)
                
            self.connection_retries += 1
            if self.connection_retries < self.max_retries:
                time.sleep(1)  # Wait before retrying
        
        if not self.onvif_connected:
            self.logger.error(f"Failed to connect to camera via ONVIF after {self.max_retries} attempts")

    # Fixed method definition with self parameter  
    def timeout_handler(self, signum, frame):
        raise TimeoutException("Camera initialization timed out")

    def _set_onvif_param(self, param: str, value, extra: tuple=None, force: bool = False):
        # Check if this parameter has been explicitly set or force is True
        if not force and param not in self.set_params:
            if self.verbose_logging:
                self.logger.info(f"Skipping {param} as it hasn't been explicitly set")
            return True
        
        if not self.onvif_connected:
            self.logger.error(f"ONVIF not connected, cannot set {param}")
            return False
            
        try:
            encoder_config = self.profile.VideoEncoderConfiguration
            imaging_settings = self.imaging_service.GetImagingSettings(self.source_token)

            if extra:
                encoder_config.Resolution.Width = extra[0]
                encoder_config.Resolution.Height = extra[1]

            if self.ONVIF_PARAMS[param] == 'imaging':
                setattr(imaging_settings, param, value)
                self.imaging_service.SetImagingSettings({
                    'VideoSourceToken': self.source_token,
                    'ImagingSettings': imaging_settings,
                    'ForcePersistence': True
                })
            elif self.ONVIF_PARAMS[param] == 'video':
                if not hasattr(encoder_config, 'Name') or encoder_config.Name is None:
                   encoder_config.Name = f"Config_{self.profile.token}"
                if not hasattr(encoder_config, 'token') or encoder_config.token is None:
                    encoder_config.token = self.profile.VideoEncoderConfiguration.token

                if param != 'Resolution':
                    setattr(encoder_config.RateControl, param, value)
                elif param == 'Resolution' and extra:
                     encoder_config.Resolution.Width = extra[0]
                     encoder_config.Resolution.Height = extra[1]

                config_dict = {
                    'Name': encoder_config.Name,
                    'token': encoder_config.token,
                    'UseCount': encoder_config.UseCount,
                    'Encoding': encoder_config.Encoding,
                    'Resolution': {
                        'Width': encoder_config.Resolution.Width,
                        'Height': encoder_config.Resolution.Height
                    },
                    'Quality': encoder_config.Quality,
                    'RateControl': {
                        'FrameRateLimit': encoder_config.RateControl.FrameRateLimit,
                        'EncodingInterval': encoder_config.RateControl.EncodingInterval,
                        'BitrateLimit': encoder_config.RateControl.BitrateLimit
                    },
                    'Multicast': {
                         'Address': {
                             'Type': encoder_config.Multicast.Address.Type,
                             'IPv4Address': encoder_config.Multicast.Address.IPv4Address,
                          },
                         'Port': encoder_config.Multicast.Port,
                         'TTL': encoder_config.Multicast.TTL,
                         'AutoStart': encoder_config.Multicast.AutoStart
                     },
                    'SessionTimeout': encoder_config.SessionTimeout
                }

                self.media_service.SetVideoEncoderConfiguration({
                    'Configuration': config_dict,
                    'ForcePersistence': True
                })

            if self.verbose_logging:
                self.logger.info(f"{param} updated to {value}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set ONVIF parameter {param}: {e}")
            return False
           
    def _set_http_param(self, param: str, value: str, extra: str = None, force: bool = False):
        # Check if this parameter has been explicitly set or force is True
        if not force and param not in self.set_params:
            if self.verbose_logging:
                self.logger.info(f"Skipping {param} as it hasn't been explicitly set")
            return True
        
        try:
            headers = {'Authorization': 'Basic YWRtaW46YWRtaW4='}
            form_data = {
                'flag': self.HTTP_PARAMS[param][1] if 'IR' not in param else '',
                self.HTTP_PARAMS[param][0]: value,
            }
            
            # Only add extra parameter if provided
            if extra:
                form_data[self.HTTP_PARAMS[param][0].replace('Text', '')] = extra
            
            url = f"http://{self.ip}/form/{'IRset' if 'IR' in param else 'CameraSet'}"
            response = requests.post(url, data=form_data, headers=headers)

            if response.status_code == 200:
                if self.verbose_logging:
                    self.logger.info(f"{param} updated to {value}")
                return True
            else:
                self.logger.error(f"Failed to update {param} via HTTP. Status: {response.status_code}")
                return False
        except Exception as e:
            self.logger.error(f"Failed to set parameter {param}: {e}")
            return False

    # ======================================================================================================================= #
    #                                                      API METHODS                                                        #
    # ======================================================================================================================= #

    def setBrightness(self, value: int):
        """
        Sets the brightness of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid brightness value: {value}")
            return False

        # Mark parameter as explicitly set
        self.set_params.add('Brightness')
        return self._set_onvif_param('Brightness', value)

    def setContrast(self, value: int):
        """
        Sets the contrast of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid contrast value: {value}")
            return False

        # Mark parameter as explicitly set
        self.set_params.add('Contrast')
        return self._set_onvif_param('Contrast', value)
    
    def setSaturation(self, value: int):
        """
        Sets the saturation of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid saturation value: {value}")
            return False

        # Mark parameter as explicitly set
        self.set_params.add('ColorSaturation')
        return self._set_onvif_param('ColorSaturation', value)
    
    def setSharpness(self, value: int):
        """
        Sets the sharpness of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid sharpness value: {value}")
            return False

        # Mark parameter as explicitly set
        self.set_params.add('Sharpness')
        return self._set_onvif_param('Sharpness', value)
    
    def setResolution(self, value: string):
        """
        Sets the resolution of the camera.
        
        Valid values: 480p, w480p, 576p, 720p, 960p, 1080p, 1536p.
        """
        if value in self.RESOLUTION_MAP:
            width, height = self.RESOLUTION_MAP[value]
            extra = (width, height)
        else:
            self.logger.error(f"Invalid resolution value: {value}")
            return False

        # Mark parameter as explicitly set
        self.set_params.add('Resolution')
        return self._set_onvif_param('Resolution', value, extra)

    def setFrameRate(self, value: FramerateValues):
        val = value.value

        # Mark parameter as explicitly set
        self.set_params.add('FrameRateLimit')
        return self._set_onvif_param('FrameRateLimit', val)
    
    def setBitrate(self, value: int):
        """
        Sets the bitrate of the camera.
        
        Valid range: 128 to 10000 (inclusive).
        """
        if not (128 <= value <= 10000):
            self.logger.error(f"Invalid bitrate value: {value}")
            return False

        # Mark parameter as explicitly set
        self.set_params.add('BitrateLimit')
        return self._set_onvif_param('BitrateLimit', value)
    
    def disableWideDynamicRange(self):
        # Mark parameter as explicitly set
        self.set_params.add('WideDynamicRange')
        return self._set_http_param('WideDynamicRange', '0')
    
    def setWideDynamicRangeLevel(self, value: int):
        """
        Sets the WDR level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid WDR value: {value}")
            return False

        # Mark parameters as explicitly set
        self.set_params.add('WideDynamicRange')
        self.set_params.add('WideDynamicRangeLevel')
        
        result1 = self._set_http_param('WideDynamicRange', '1')
        result2 = self._set_http_param('WideDynamicRangeLevel', str(value))
        return result1 and result2
    
    def enableBackLight(self):
        # Mark parameter as explicitly set
        self.set_params.add('BackLight')
        return self._set_http_param('BackLight', '1')
    
    def disableBackLight(self):
        # Mark parameter as explicitly set
        self.set_params.add('BackLight')
        return self._set_http_param('BackLight', '0')
    
    def HorizontalMirror(self):
        # Mark parameter as explicitly set
        self.set_params.add('MirrorHorizontal')
        return self._set_http_param('MirrorHorizontal', '1')
    
    def VerticalMirror(self):
        # Mark parameter as explicitly set
        self.set_params.add('MirrorVertical')
        return self._set_http_param('MirrorVertical', '1')
    
    def resetHorizontalMirror(self):
        # Mark parameter as explicitly set
        self.set_params.add('MirrorHorizontal')
        return self._set_http_param('MirrorHorizontal', '0')
    
    def resetVerticalMirror(self):
        # Mark parameter as explicitly set
        self.set_params.add('MirrorVertical')
        return self._set_http_param('MirrorVertical', '0')

    def enableAntiFalseColor(self):
        # Mark parameter as explicitly set
        self.set_params.add('AntiFalseColor')
        return self._set_http_param('AntiFalseColor', '1')
    
    def disableAntiFalseColor(self):
        # Mark parameter as explicitly set
        self.set_params.add('AntiFalseColor')
        return self._set_http_param('AntiFalseColor', '0')
    
    def enableDigitalImageStabilizer(self):
        # Mark parameter as explicitly set
        self.set_params.add('DigitalImageStabilizer')
        return self._set_http_param('DigitalImageStabilizer', '1')
    
    def disableDigitalImageStabilizer(self):
        # Mark parameter as explicitly set
        self.set_params.add('DigitalImageStabilizer')
        return self._set_http_param('DigitalImageStabilizer', '0')
    
    def enableLensShadeCorrection(self):
        # Mark parameter as explicitly set
        self.set_params.add('LensShadeCorrection')
        return self._set_http_param('LensShadeCorrection', '1')
    
    def disableLensShadeCorrection(self):
        # Mark parameter as explicitly set
        self.set_params.add('LensShadeCorrection')
        return self._set_http_param('LensShadeCorrection', '0')
    
    def setLensDistortionCorrection(self, value: int):
        """
        Sets the Lens distortion correction level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid LDC value: {value}")
            return False

        # Mark parameters as explicitly set
        self.set_params.add('LensDistortionCorrection')
        self.set_params.add('LensDistortionCorrectionLevel')
        
        result1 = self._set_http_param('LensDistortionCorrection', '1')
        result2 = self._set_http_param('LensDistortionCorrectionLevel', str(value))
        return result1 and result2
    
    def disableLensDistortionCorrection(self):
        # Mark parameter as explicitly set
        self.set_params.add('LensDistortionCorrection')
        return self._set_http_param('LensDistortionCorrection', '0')
    
    def setAntiFog(self, value: int):
        """
        Sets the Antifog level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid Anti-fog level value: {value}")
            return False

        # Mark parameters as explicitly set
        self.set_params.add('AntiFog')
        self.set_params.add('AntiFogLevel')
        
        result1 = self._set_http_param('AntiFog', '1')
        result2 = self._set_http_param('AntiFogLevel', str(value))
        return result1 and result2
    
    def disableAntiFog(self):
        # Mark parameter as explicitly set
        self.set_params.add('AntiFog')
        return self._set_http_param('AntiFog', '0')
    
    def setScene(self, mode: Scenes = Scenes.INDOOR):
        value = mode.value
        
        # Mark parameter as explicitly set
        self.set_params.add('SceneSelect')
        return self._set_http_param('SceneSelect', str(value))
    
    def setExposureMode(self, mode: ExposureModes = ExposureModes.MANUAL):
        value = mode.value
        
        # Mark parameter as explicitly set
        self.set_params.add('ExposureMode')
        return self._set_http_param('ExposureMode', str(value))
    
    def setShutterSpeed(self, value: ShutterValues = ShutterValues._1_50):
        val = value.value
        
        # Mark parameter as explicitly set
        self.set_params.add('ShutterSpeed')
        
        value_str = value.name[1:].replace('_', '/')
        extra = str(value.value)
        return self._set_http_param('ShutterSpeed', value_str, extra)

    def setManualACG(self, value: AEGains = AEGains._1X):
        val = value.value
        
        # Mark parameter as explicitly set
        self.set_params.add('ManualACG')
        
        value_str = value.name[1:].replace('_', '/')
        extra = str(value.value)
        return self._set_http_param('ManualACG', value_str, extra)

    def setWhiteBalanceMode(self, mode: WhiteBalanceModes = WhiteBalanceModes.AUTO):
        value = mode.value
        
        # Mark parameter as explicitly set
        self.set_params.add('WhiteBalanceModeSelect')
        return self._set_http_param('WhiteBalanceModeSelect', str(value))
    
    def setIRMode(self, mode: IRModes = IRModes.AUTO):
        value = mode.value
        
        # Mark parameters as explicitly set
        self.set_params.add('IRenable')
        self.set_params.add('IRmode')
        
        result1 = self._set_http_param('IRenable', '1')
        result2 = self._set_http_param('IRmode', str(value))
        return result1 and result2
    
    def disableIR(self):
        # Mark parameter as explicitly set
        self.set_params.add('IRenable')
        return self._set_http_param('IRenable', '0')
        
    # Helper method to check if a parameter has been explicitly set
    def isParameterSet(self, param_name):
        return param_name in self.set_params
        
    # Helper method to reset parameter tracking
    def resetParameters(self):
        self.set_params.clear()