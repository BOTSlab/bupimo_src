import rospy
from op import Op
import cast_obstacle_detector
import settings

from picamera_ops.msg import *

class CastObstacleDetectorOp(Op):

    def __init__(self, settings):
        self.debug = settings.debug
        cast_obstacle_detector.init(settings)
        self.publisher = rospy.Publisher('castobstacles', CastObstacleArray, \
                                          queue_size=1)

    def apply(self, image, image_debug):
        # Detect cast obstacles
        msg = CastObstacleArray()
        msg.obstacles = cast_obstacle_detector.find_obstacles_on_all_lines( \
                                                image, image_debug, self.debug)
        self.publisher.publish(msg)

