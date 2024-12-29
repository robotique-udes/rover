import requests, signal, string
from onvif import ONVIFCamera
from enum import Enum

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

    def __init__(self, node, ip: str, port: int, username: str, password: str, timeout: int = 5):
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self.logger = node.get_logger()

        signal.signal(signal.SIGALRM, self.timeout_handler)
        signal.alarm(timeout)

        try:
            self.camera = ONVIFCamera(self.ip, self.port, self.username, self.password)

            self.media_service = self.camera.create_media_service()
            profiles = self.media_service.GetProfiles()
            if not profiles:
                raise Exception("No media profiles found on the camera")
            self.profile = profiles[0]

            self.imaging_service = self.camera.create_imaging_service()
            self.source_token = self.profile.VideoSourceConfiguration.SourceToken

        except TimeoutException:
            self.logger.error("Camera initialization timed out")
        except Exception as e:
            self.logger.error(f"Failed to initialize the camera: {str(e).split(':')[-1].strip()}")
        finally:
            signal.alarm(0)  

    def _timeout_handler(signum, frame):
            raise TimeoutException("Camera initialization timed out")

    def _set_onvif_param(self, param: str, value: str, extra: tuple=None):
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
                setattr(encoder_config.RateControl, param, value)
                self.media_service.SetVideoEncoderConfiguration({
                    'Configuration': encoder_config,
                    'ForcePersistence': True
                })
            self.logger.info(f"{param} updated to {value}")
        except Exception as e:
            self.logger.error(f"Failed to set ONVIF parameter {param}: {e}")
           
    def _set_http_param(self, param: str, value: str, extra: str = None):
        try:
            headers = {'Authorization': 'Basic YWRtaW46YWRtaW4='}
            form_data = {
                'flag': self.HTTP_PARAMS[param][1] if 'IR' not in param else '',
                self.HTTP_PARAMS[param][0]: value,
                self.HTTP_PARAMS[param][0].replace('Text', ''): extra if extra else ''
            }

            url = f"http://{self.ip}/form/{'IRset' if 'IR' in param else 'CameraSet'}"
            response = requests.post(url, data=form_data, headers=headers)

            if response.status_code == 200:
                self.logger.info(f"{param} updated to {value}")
            else:
                self.logger.error(f"Failed to update {param} via HTTP. Status: {response.status_code}")
        except Exception as e:
            self.logger.error(f"Failed to set parameter {param}: {e}")

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

        self._set_onvif_param('Brightness', value)

    def setContrast(self, value: int):
        """
        Sets the contrast of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """

        if not (0 <= value <= 255):
            self.logger.error(f"Invalid contrast value: {value}")
            return False

        self._set_onvif_param('Contrast', value)
    
    def setSaturation(self, value: int):
        """
        Sets the saturation of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """

        if not (0 <= value <= 255):
            self.logger.error(f"Invalid saturation value: {value}")
            return False

        self._set_onvif_param('ColorSaturation', value)
    
    def setSharpness(self, value: int):
        """
        Sets the sharpness of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """

        if not (0 <= value <= 255):
            self.logger.error(f"Invalid sharpness value: {value}")
            return False

        self._set_onvif_param('Sharpness', value)
    
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

        self._set_onvif_param('Resolution', value, extra)

    def setFrameRate(self, value: FramerateValues):
        self._set_onvif_param('FrameRateLimit', value.value)
    
    def setBitrate(self, value: int):
        """
        Sets the bitrate of the camera.
        
        Valid range: 128 to 10000 (inclusive).
        """

        if not (128 <= value <= 10000):
            self.logger.error(f"Invalid bitrate value: {value}")
            return False

        self._set_onvif_param('BitrateLimit', value)
    
    def disableWideDynamicRange(self):
        self._set_http_param('WideDynamicRange', '0')
    
    def setWideDynamicRangeLevel(self, value: int):
        """
        Sets the WDR level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """

        if not (0 <= value <= 255):
            self.logger.error(f"Invalid WDR value: {value}")
            return False

        self._set_http_param('WideDynamicRange', '1')
        self._set_http_param('WideDynamicRangeLevel', value)
    
    def enableBackLight(self):
        self._set_http_param('BackLight', '1')
    
    def disableBackLight(self):
        self._set_http_param('BackLight', '0')
    
    def HorizontalMirror(self):
        self._set_http_param('MirrorHorizontal', '1')
    
    def VerticalMirror(self):
        self._set_http_param('MirrorVertical', '1')
    
    def resetHorizontalMirror(self):
        self._set_http_param('MirrorHorizontal', '0')
    
    def resetVerticalMirror(self):
        self._set_http_param('MirrorVertical', '0')

    def enableAntiFalseColor(self):
        self._set_http_param('AntiFalseColor', '1')
    
    def disableAntiFalseColor(self):
        self._set_http_param('AntiFalseColor', '0')
    
    def enableDigitalImageStabilizer(self):
        self._set_http_param('DigitalImageStabilizer', '1')
    
    def disableDigitalImageStabilizer(self):
        self._set_http_param('DigitalImageStabilizer', '0')
    
    def enableLensShadeCorrection(self):
        self._set_http_param('LensShadeCorrection', '1')
    
    def disableLensShadeCorrection(self):
        self._set_http_param('LensShadeCorrection', '0')
    
    def setLensDistortionCorrection(self, value: int):
        """
        Sets the Lens distortion correction level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """

        if not (0 <= value <= 255):
            self.logger.error(f"Invalid LDC value: {value}")
            return False

        self._set_http_param('LensDistortionCorrection', '1')
        self._set_http_param('LensDistortionCorrectionLevel', value)
    
    def disableLensDistortionCorrection(self):
        self._set_http_param('LensDistortionCorrection', '0')
    
    def setAntiFog(self, value: int):
        """
        Sets the Antifog level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """

        if not (0 <= value <= 255):
            self.logger.error(f"Invalid Anti-fog level value: {value}")
            return False

        self._set_http_param('AntiFog', '1')
        self._set_http_param('AntiFogLevel', value)
    
    def disableAntiFog(self):
        self._set_http_param('AntiFog', '0')
    
    def setScene(self, mode: Scenes = Scenes.INDOOR.name):
        self._set_http_param('SceneSelect', mode.value)
    
    def setExposureMode(self, mode: ExposureModes = ExposureModes.MANUAL.name):
        self._set_http_param('ExposureMode', mode.value)
    
    def setShutterSpeed(self, value: ShutterValues = ShutterValues._1_50.name):
        value = value.name[1:].replace('_', '/')
        extra = value.value
        self._set_http_param('ShutterSpeed', value, extra)

    def setManualACG(self, value: AEGains = AEGains._1X.name):
        value = value.name[1:].replace('_', '/')
        extra = value.value
        self._set_http_param('AEGainText', value, extra)

    def setWhiteBalanceMode(self, mode: WhiteBalanceModes = WhiteBalanceModes.AUTO.name):
        self._set_http_param('WhiteBalanceModeSelect', mode.value)
    
    def setIRMode(self, mode: IRModes = IRModes.AUTO.name):
        self._set_http_param('IRenable', '1')
        self._set_http_param('IRmode', mode.value)
    
    def disableIR(self):
        self._set_http_param('IRenable', '0')