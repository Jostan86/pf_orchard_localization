from .data_loader import Ros1Bag, Ros2Bag, Cached

from typing import Union
AnyDataLoader = Union[Ros1Bag, Ros2Bag, Cached]

