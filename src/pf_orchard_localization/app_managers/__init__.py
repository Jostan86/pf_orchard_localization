from .base import Base
from .cached import Cached
from .live import Live
from .ros_bags import RosBags

from typing import Union
AnyManager = Union[Cached, Live, RosBags]