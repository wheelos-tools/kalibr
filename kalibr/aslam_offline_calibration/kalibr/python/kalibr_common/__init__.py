# Import the numpy to Eigen type conversion.
import numpy_eigen
from .ConfigReader import *
from .FolderImageDatasetReader import *
from .TargetExtractor import *

# ROS bag support is optional in the Apollo PNG-only runtime.  Keep exporting
# the historical readers whenever their ROS Python dependencies are present.
try:
    from .ImageDatasetReader import *
    from .ImuDatasetReader import *
except ModuleNotFoundError as error:
    if error.name not in ("cv_bridge", "rosbag"):
        raise
