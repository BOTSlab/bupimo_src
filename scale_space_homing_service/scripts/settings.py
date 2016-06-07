import rospy


def init(debugIn = False):
    global xRes
    global yRes
    global roiCenter
    global inner_rad
    global outer_rad
    global debug

    debug = debugIn    

    xRes = rospy.get_param("image_width")
    yRes = rospy.get_param("image_height")
    roiCenter = (rospy.get_param("image_mirror_centre_x"), rospy.get_param("image_mirror_centre_y"))
    inner_rad = rospy.get_param("image_mirror_inner_radius")
    outer_rad = rospy.get_param("image_mirror_outer_radius")
