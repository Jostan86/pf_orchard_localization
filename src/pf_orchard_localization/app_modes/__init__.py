from .pf_recorded_data import PfRecordedData, PfCached, PfCachedTests, PfSaveCalibrationData
from .image_playback import ImagePlayback
from .pf_live import PfLive

from typing import Union

AnyMode = Union[
            ImagePlayback, 
            PfCached, 
            PfCachedTests, 
            PfLive, 
            PfRecordedData, 
            PfSaveCalibrationData
            ]
"""AnyMode: A type hint for any of the app modes in the localization app."""