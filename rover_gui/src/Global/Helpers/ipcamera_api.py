import requests
import string
import logging
from onvif import ONVIFCamera
from enum import Enum

logger = logging.getLogger(__name__) 

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

class ParameterHandler:
    
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
    
    def __init__(self, ip: str, port: int, username: str, password: str):
        self.logger = logger
        self.verbose_logging = False
        
        # Configure basic logging if not already configured
        if not logging.getLogger().handlers:
            logging.basicConfig(level=logging.INFO, 
                               format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self.camera = ONVIFCamera(self.ip, self.port, self.username, self.password)
        self.media_service = self.camera.create_media_service()
        self.profiles = self.media_service.GetProfiles()
        self.profile = self.profiles[0]
        self.imaging_service = self.camera.create_imaging_service()
        self.source_token = self.profile.VideoSourceConfiguration.SourceToken

    def _set_onvif_param(self, param: str, value, extra: tuple=None):
            
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
        
        try:
            headers = {'Authorization': 'Basic YWRtaW46YWRtaW4='}
            form_data = {
                'flag': self.HTTP_PARAMS[param][1] if 'IR' not in param else '',
                self.HTTP_PARAMS[param][0]: value,
            }
            
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

        return self._set_onvif_param('Brightness', value)

    def setContrast(self, value: int):
        """
        Sets the contrast of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid contrast value: {value}")
            return False

        return self._set_onvif_param('Contrast', value)
    
    def setSaturation(self, value: int):
        """
        Sets the saturation of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid saturation value: {value}")
            return False

        return self._set_onvif_param('ColorSaturation', value)
    
    def setSharpness(self, value: int):
        """
        Sets the sharpness of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid sharpness value: {value}")
            return False

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

        return self._set_onvif_param('Resolution', value, extra)

    def setFrameRate(self, value: FramerateValues):
        val = value.value

        return self._set_onvif_param('FrameRateLimit', val)
    
    def setBitrate(self, value: int):
        """
        Sets the bitrate of the camera.
        
        Valid range: 128 to 10000 (inclusive).
        """
        if not (128 <= value <= 10000):
            self.logger.error(f"Invalid bitrate value: {value}")
            return False

        return self._set_onvif_param('BitrateLimit', value)
    
    def disableWideDynamicRange(self):

        return self._set_http_param('WideDynamicRange', '0')
    
    def setWideDynamicRangeLevel(self, value: int):
        """
        Sets the WDR level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid WDR value: {value}")
            return False
        
        result1 = self._set_http_param('WideDynamicRange', '1')
        result2 = self._set_http_param('WideDynamicRangeLevel', str(value))
        return result1 and result2
    
    def enableBackLight(self):

        return self._set_http_param('BackLight', '1')
    
    def disableBackLight(self):

        return self._set_http_param('BackLight', '0')
    
    def HorizontalMirror(self):

        return self._set_http_param('MirrorHorizontal', '1')
    
    def VerticalMirror(self):

        return self._set_http_param('MirrorVertical', '1')
    
    def resetHorizontalMirror(self):

        return self._set_http_param('MirrorHorizontal', '0')
    
    def resetVerticalMirror(self):
     
        return self._set_http_param('MirrorVertical', '0')

    def enableAntiFalseColor(self):

        return self._set_http_param('AntiFalseColor', '1')
    
    def disableAntiFalseColor(self):

        return self._set_http_param('AntiFalseColor', '0')
    
    def enableDigitalImageStabilizer(self):

        return self._set_http_param('DigitalImageStabilizer', '1')
    
    def disableDigitalImageStabilizer(self):

        return self._set_http_param('DigitalImageStabilizer', '0')
    
    def enableLensShadeCorrection(self):

        return self._set_http_param('LensShadeCorrection', '1')
    
    def disableLensShadeCorrection(self):

        return self._set_http_param('LensShadeCorrection', '0')
    
    def setLensDistortionCorrection(self, value: int):
        """
        Sets the Lens distortion correction level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid LDC value: {value}")
            return False

        result1 = self._set_http_param('LensDistortionCorrection', '1')
        result2 = self._set_http_param('LensDistortionCorrectionLevel', str(value))
        return result1 and result2
    
    def disableLensDistortionCorrection(self):

        return self._set_http_param('LensDistortionCorrection', '0')
    
    def setAntiFog(self, value: int):
        """
        Sets the Antifog level of the camera.
        
        Valid range: 0 to 255 (inclusive).
        """
        if not (0 <= value <= 255):
            self.logger.error(f"Invalid Anti-fog level value: {value}")
            return False

        result1 = self._set_http_param('AntiFog', '1')
        result2 = self._set_http_param('AntiFogLevel', str(value))
        return result1 and result2
    
    def disableAntiFog(self):

        return self._set_http_param('AntiFog', '0')
    
    def setScene(self, mode: Scenes = Scenes.INDOOR):
        value = mode.value
        
        return self._set_http_param('SceneSelect', str(value))
    
    def setExposureMode(self, mode: ExposureModes = ExposureModes.MANUAL):
        value = mode.value
        
        return self._set_http_param('ExposureMode', str(value))
    
    def setShutterSpeed(self, value: ShutterValues = ShutterValues._1_50):
        
        value_str = value.name[1:].replace('_', '/')
        extra = str(value.value)
        return self._set_http_param('ShutterSpeed', value_str, extra)

    def setManualACG(self, value: AEGains = AEGains._1X):
        
        value_str = value.name[1:].replace('_', '/')
        extra = str(value.value)
        return self._set_http_param('ManualACG', value_str, extra)

    def setWhiteBalanceMode(self, mode: WhiteBalanceModes = WhiteBalanceModes.AUTO):
        value = mode.value
        
        return self._set_http_param('WhiteBalanceModeSelect', str(value))
    
    def setIRMode(self, mode: IRModes = IRModes.AUTO):
        value = mode.value
        
        result1 = self._set_http_param('IRenable', '1')
        result2 = self._set_http_param('IRmode', str(value))
        return result1 and result2
    
    def disableIR(self):

        return self._set_http_param('IRenable', '0')