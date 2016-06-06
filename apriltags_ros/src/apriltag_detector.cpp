/**
 * Apriltag publisher: Subscribes to 'small_image_topic' and publishes detected
 * tags on 'aprilTags'.  This originally came from the RiverLab and was further
 * modified by Dalia Said.  I have tailored this so that the published tags
 * contain only those fields which we can actually measure without careful
 * camera calibration.  Also, the assumption is that the tags appear in an
 * omnidirectional image.  The centre of the mirror is specified through
 * parameters on the ROS parameter server.
 *
 * Andrew Vardy
 * 5 June, 2016
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <AprilTags/TagDetector.h>
#include <cv_bridge/cv_bridge.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

ros::Publisher detections_pub;

float imgCenterX = 238.0; //370.0;
float imgCenterY = 129.0; //350.0;

void imageCallback(const sensor_msgs::ImageConstPtr &image) {

	AprilTags::TagCodes tag_codes(AprilTags::tagCodes16h5);	
	AprilTags::TagDetector* tag_detector = 
                                        new AprilTags::TagDetector(tag_codes);

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
	cv::Mat image_gray;
	cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
	
	vector<AprilTags::TagDetection> detections =
                                        tag_detector->extractTags(image_gray);
	apriltags_ros::AprilTagDetectionArray tag_detection_array;
	for (int i = 0; i < detections.size(); i++) {

		apriltags_ros::AprilTagDetection tag_detection;
		tag_detection.id = detections[i].id;
		tag_detection.x = detections[i].cxy.first; //pixel x-location
		tag_detection.y = detections[i].cxy.second; //pixel y-location
		tag_detection.code = detections[i].code;

        // The maximum side length is used as a proxy for the apparent size
        // of the tag.  Area would be a more obvious measure, but that would
        // be more sensitive to changes in orientation (as opposed to just
        // distance which is what we want to measure).
        std::pair<float, float> *p = &(detections[i].p);
        float s1 = sqrt(pow((p[1].first - p[0].first), 2) +
                        pow((p[1].second - p[0].second), 2));
        float s2 = sqrt(pow((p[2].first - p[1].first), 2) +
                        pow((p[2].second - p[1].second), 2));
        float s3 = sqrt(pow((p[3].first - p[2].first), 2) +
                        pow((p[3].second - p[2].second), 2));
        float s4 = sqrt(pow((p[0].first - p[3].first), 2) +
                        pow((p[0].second - p[3].second), 2));
        if (s1 >= s2 && s1 >= s3 && s1 >= s4)
            tag_detection.max_side_length = s1;
        else if (s2 >= s1 && s2 >= s3 && s2 >= s4)
            tag_detection.max_side_length = s2;
        else if (s3 >= s1 && s3 >= s2 && s3 >= s4)
            tag_detection.max_side_length = s3;
        else
            tag_detection.max_side_length = s4;

        // Distance in pixels from the image centre.  Not meaningful as a
        // spatial distance, but potentially useful as a measure of apparent
        // size.
		tag_detection.distance_from_image_centre = 
            sqrt(pow((imgCenterX - tag_detection.x), 2) +
                 pow((imgCenterY - tag_detection.y), 2));

		tag_detection.bearing = atan2((tag_detection.y - imgCenterY),
                                      (tag_detection.x - imgCenterX));

		tag_detection_array.detections.push_back(tag_detection);

	}

    if (debug)
        for (int i = 0; i < detections.size(); i++)
            detections[i].draw(image_gray);
        cv::waitKey(10);

	detections_pub.publish(tag_detection_array);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "apriltag_detector");
	ros::NodeHandle n;

    n.getParam("image_mirror_centre_x", imgCenterX);
    n.getParam("image_mirror_centre_y", imgCenterY);

	ros::Subscriber imageSubscriber = n.subscribe("small_image_topic", 
                                                  1, imageCallback);
	detections_pub = n.advertise<apriltags_ros::AprilTagDetectionArray>(
                                                  "aprilTags", 1);
	ros::Rate r(10);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	return(0);
}
