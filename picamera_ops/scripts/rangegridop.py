import rospy, cv2, math
from op import Op
import numpy as np
import settings
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class RangeGridOp(Op):

    GRID_WIDTH = 100
    GRID_HEIGHT = 100

    def __init__(self, settings):
        self.debug = settings.debug
        #self.publisher = rospy.Publisher('castobstacles', CastObstacleArray, \
        #                                  queue_size=1)

        # Parameters of the image
        self.width = settings.xRes
        self.height = settings.yRes
        self.cx = settings.roiCenter[0]
        self.cy = settings.roiCenter[1]
        self.outer_rad = settings.outer_rad

        # Create interpolating function 'pixels2range'
        image_dist = [0, \
             settings.image_mirror_30cm_radius, \
             settings.image_mirror_60cm_radius, 
             settings.image_mirror_90cm_radius ]
        spatial_dist = [0, 0.3, 0.6, 0.9]
        self.pixels2range = interp1d(image_dist, spatial_dist)

        # Do a little test plot
        i = np.arange(0, settings.image_mirror_90cm_radius, 1)
        plt.plot(image_dist, spatial_dist, 'o', i, self.pixels2range(i), '-')
        plt.show()
        

        # We now need to determine the extent of space around the robot
        # that needs to be represented.  We do this by using the corners of
        # image and calculating their corresponding coordinates.
        """
        x0, y0 = self.image2space(0, 0)
        x1, y1 = self.image2space(self.width-1, 0)
        x2, y2 = self.image2space(self.width-1, self.height-1)
        x3, y3 = self.image2space(0, self.height-1)
        self.xr_min = min([x0, x1, x2, x3])
        self.xr_max = max([x0, x1, x2, x3])
        self.yr_min = min([y0, y1, y2, y3])
        self.yr_max = max([y0, y1, y2, y3])
        """
        self.xr_min = float('inf')
        self.xr_max = -float('inf')
        self.yr_min = float('inf')
        self.yr_max = -float('inf')
        for yi in range(self.height):
            print "yi: {}".format(yi)
            for xi in range(self.width):
                #print "xi, yi: {}, {}".format(xi, yi)
                xr, yr = self.image2space(xi, yi)
                #print "xr, yr: {}, {}".format(xr, yr)
                self.xr_min = min(self.xr_min, xr)
                self.xr_max = max(self.xr_max, xr)
                self.yr_min = min(self.yr_min, yr)
                self.yr_max = max(self.yr_max, yr)
                
        self.xr_range = self.xr_max - self.xr_min
        self.yr_range = self.yr_max - self.yr_min
        print "xr_min: {}".format(self.xr_min)
        print "xr_max: {}".format(self.xr_max)
        print "yr_min: {}".format(self.yr_min)
        print "yr_max: {}".format(self.yr_max)

        # Create the range grid
        self.grid = np.zeros((self.GRID_HEIGHT, self.GRID_WIDTH), np.uint8)

    def image2space(self, xi, yi):
        """Convert from image coordinates (xi, yi) to coordinates in space in
           the robot reference frame."""

        # alpha and rho are polar coordinates of point (xi, yi) with respect to
        # the centre of the mirror.  alpha = 0 is to the right of the image
        # and is aligned with the forwards direction.
        dy = self.cy - yi
        dx = xi - self.cx
        alpha = math.atan2(dy, dx)
        rho = math.sqrt(dx*dx + dy*dy)

        distance = self.pixels2range(rho)

        # Now the (xr, yr) coordinates.
        xr = distance * math.cos(alpha)
        yr = distance * math.sin(alpha)

        return xr, yr
        
    def space2grid(self, xr, yr):
        """Convert from coordinates in the robot reference frame to the grid."""
        
        xg = int((self.GRID_WIDTH - 1) * ((xr - self.xr_min) / self.xr_range))
        yg = int((self.GRID_HEIGHT - 1) * ((yr - self.yr_min) / self.yr_range))
        
        return xg, yg

    def apply(self, image, image_debug):

        #if self.debug:
        #    image_debug = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        self.grid.fill(0)

        print("width: " + str(len(image[0])))
        print("height: " + str(len(image)))
        assert(self.width == len(image[0]))
        assert(self.height == len(image))

        for yi in range(self.height):
            for xi in range(self.width):
                dx = xi - self.cx
                dy = yi - self.cy
                radius = math.sqrt(dx*dx + dy*dy)
                if radius < self.outer_rad:
                    if image[yi][xi] > 20:
                        #print "xi, yi: {}, {}".format(xi, yi)
                        xr, yr = self.image2space(xi, yi)
                        #print "xr, yr: {}, {}".format(xr, yr)
                        xg, yg = self.space2grid(xr, yr)
                        #print "xg, yg: {}, {}".format(xg, yg)
                        self.grid[yg, xg] = 255

        if self.debug:
            #image_debug = cv2.cvtColor(image_debug, cv2.COLOR_GRAY2BGR)
            cv2.imshow("RangeGridOp", self.grid)
            key = cv2.waitKey(1) & 0xFF

