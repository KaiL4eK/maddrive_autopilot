from __future__ import print_function

import cv2

window_name = 'roi'

def image_callback(ros_data):
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow(window_name, cv_image)
    cv2.waitKey(3)

def main():
	rospy.init_node('roi_tuner')
	subscriber = rospy.Subscriber('/camera/rgb/image_color', Image, callback,  queue_size = 10)

	cv2.namedWindow(window_name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
