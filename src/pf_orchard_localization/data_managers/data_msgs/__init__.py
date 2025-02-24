from .data_msgs import Gnss, Imu, Image, Odom, MsgType, Source, PoseEstimate, Timestamp

from typing import Union
AnyDataMsg = Union[
                    Gnss,
                    Image,
                    Odom,
                    PoseEstimate,
                    Imu
                    ]