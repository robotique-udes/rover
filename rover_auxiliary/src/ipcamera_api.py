import logging, requests, signal, string
from onvif import ONVIFCamera

class TimeoutException(Exception):
    pass

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

    SHUTTER_MAP = {
        "1/8000": 0,
        "1/6000": 1,
        "1/4000": 2,
        "1/2000": 3,
        "1/1000": 4,
        "1/500": 5,
        "1/250": 6,
        "1/200": 7,
        "1/150": 8,
        "1/100": 9,
        "1/50": 10,
        "1/25": 11,
        "1/20": 12,
        "1/15": 13,
        "1/10": 14,
        "1/8": 15,
        "1/5": 16,
        "1/3": 17,
        "1/2": 18,
        "1": 19
    }

    AEGAIN_MAP = {
        "1X": 0,
        "2X": 1,
        "4X": 2,
        "8X": 3,
        "16X": 4,
        "32X": 5,
        "64X": 6
    }

    def __init__(self, ip: str, port: int, username: str, password: str, timeout: int = 5):
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password

        # Configure logging
        for handler in logging.root.handlers[:]:
            logging.root.removeHandler(handler)
        logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(message)s')
        self.logger = logging.getLogger()

        def timeout_handler(signum, frame):
            raise TimeoutException("Camera initialization timed out")

        signal.signal(signal.SIGALRM, timeout_handler)
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

    def set_onvif_param(self, param: str, value: str, extra: tuple=None):
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
           
    def set_http_param(self, param: str, value: str, extra: str = None):
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
        self.set_onvif_param('Brightness', value)

    def setContrast(self, value: int):
        self.set_onvif_param('Contrast', value)
    
    def setSaturation(self, value: int):
        self.set_onvif_param('ColorSaturation', value)
    
    def setSharpness(self, value: int):
        self.set_onvif_param('Sharpness', value)
    
    def setResolution(self, value: string):
        if value in self.RESOLUTION_MAP:
            width, height = self.RESOLUTION_MAP[value]
            extra = (width, height)
        else:
            self.logger.error(f"Invalid resolution value: {value}")
            return False

        self.set_onvif_param('Resolution', value, extra)

    def setFrameRate(self, value: int):
        self.set_onvif_param('FrameRateLimit', value)
    
    def setBitrate(self, value: int):
        self.set_onvif_param('BitrateLimit', value)
    
    def disableWideDynamicRange(self):
        self.set_http_param('WideDynamicRange', '0')
    
    def setWideDynamicRangeLevel(self, value: int = 50):
        self.set_http_param('WideDynamicRange', '1')
        self.set_http_param('WideDynamicRangeLevel', value)
    
    def enableBackLight(self):
        self.set_http_param('BackLight', '1')
    
    def disableBackLight(self):
        self.set_http_param('BackLight', '0')
    
    def HorizontalMirror(self):
        self.set_http_param('MirrorHorizontal', '1')
    
    def VerticalMirror(self):
        self.set_http_param('MirrorVertical', '1')
    
    def resetHorizontalMirror(self):
        self.set_http_param('MirrorHorizontal', '0')
    
    def resetVerticalMirror(self):
        self.set_http_param('MirrorVertical', '0')

    def enableAntiFalseColor(self):
        self.set_http_param('AntiFalseColor', '1')
    
    def disableAntiFalseColor(self):
        self.set_http_param('AntiFalseColor', '0')
    
    def enableDigitalImageStabilizer(self):
        self.set_http_param('DigitalImageStabilizer', '1')
    
    def disableDigitalImageStabilizer(self):
        self.set_http_param('DigitalImageStabilizer', '0')
    
    def enableLensShadeCorrection(self):
        self.set_http_param('LensShadeCorrection', '1')
    
    def disableLensShadeCorrection(self):
        self.set_http_param('LensShadeCorrection', '0')
    
    def setLensDistortionCorrection(self, value: int = 50):
        self.set_http_param('LensDistortionCorrection', '1')
        self.set_http_param('LensDistortionCorrectionLevel', value)
    
    def disableLensDistortionCorrection(self):
        self.set_http_param('LensDistortionCorrection', '0')
    
    def setAntiFog(self, value: int = 50):
        self.set_http_param('AntiFog', '1')
        self.set_http_param('AntiFogLevel', value)
    
    def disableAntiFog(self):
        self.set_http_param('AntiFog', '0')
    
    def setScene(self, value = 'indoor'):
        if value == 'outdoor':
            value = 0
        elif value == 'indoor':
            value = 2
        else:
            self.logger.info('Invalid scene value')
            return
        self.set_http_param('SceneSelect', value)
    
    def setExposureMode(self, value = 'scene'):
        if value == 'scene':
            value = 0
        elif value == 'manual':
            value = 1
        elif value == 'shutter':
            value = 2
        else:
            self.logger.info('Invalid exposure mode value')
            return
        self.set_http_param('ExposureMode', value)
    
    def setShutterSpeed(self, value = '1/100'):
        if value in self.SHUTTER_MAP:
            extra = self.SHUTTER_MAP[value]
        else:
            self.logger.info('Invalid shutter speed')
            return
        self.set_http_param('ShutterSpeed', value, extra)

    def setManualACG(self, value = '2X'):
        if value in self.AEGAIN_MAP:
            extra = self.AEGAIN_MAP[value]
        else:
            self.logger.info('Invalid ACG value')
            return
        self.set_http_param('AEGainText', value, extra)

    def setWhiteBalanceMode(self, value = 'auto'):
        if value == 'manual':
            value = 1
        elif value == 'auto':
            value = 0
        elif value == 'indoor':
            value = 8
        elif value == 'outdoor':
            value = 9
        elif value == 'sunlight':
            value = 2
        else:
            self.logger.info('Invalid white balance mode')
            return
        self.set_http_param('WhiteBalanceModeSelect', value)
    
    def setIRMode(self, value = 'auto'):
        self.set_http_param('IRenable', '1')
        if value == 'auto':
            value = 4
        elif value == 'day':
            value = 3
        elif value == 'night':
            value = 2
        else:
            self.logger.info('Invalid IR mode')
            return
        self.set_http_param('IRmode', value)
    
    def disableIR(self):
        self.set_http_param('IRenable', '0')
