import rospy

def init():
    global debug
    global xRes
    global yRes
    global roiCenter
    global inner_rad
    global outer_rad
#    global horizon
#    global image_mirror_30cm_radius
#    global image_mirror_60cm_radius
#    global image_mirror_90cm_radius

    debug = rospy.get_param("debug")
    xRes = rospy.get_param("image_width")
    yRes = rospy.get_param("image_height")
    roiCenter = (rospy.get_param("image_mirror_centre_x"), rospy.get_param("image_mirror_centre_y"))
    inner_rad = rospy.get_param("image_mirror_inner_radius")
    outer_rad = rospy.get_param("image_mirror_outer_radius")
#    horizon = rospy.get_param("image_mirror_horizon_radius")
#
#    image_mirror_30cm_radius = rospy.get_param('image_mirror_30cm_radius')
#    image_mirror_60cm_radius = rospy.get_param('image_mirror_60cm_radius')
#    image_mirror_90cm_radius = rospy.get_param('image_mirror_90cm_radius')
