You will need an image from the camera to be calibrated.  The name of this image and the parameters of the targets in the image should be specified in common_calib.py.

First run one of the following (read their comments to decide which):

    1. correspondences_from_picks.py

    2. correspondences_from_tags.py (executed through ROS)
        - Actually, this is intended to work with AprilTags.  Not tested or
          used recently.  So you should probably just use 1.

Then run interpolator.py.  All files read and written are local to the current
directory

28 May, 2016
Andrew Vardy
