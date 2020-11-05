import logging
import sys
import numpy as np
import rospy
import rosgraph.roslogging as rl

from cv_bridge import CvBridge, CvBridgeError

from autolab_core import Point, Logger
from visualization import Visualizer2D as vis
from perception import BinaryImage, CameraIntrinsics, ColorImage, DepthImage

from gqcnn.msg import GQCNNGrasp
from sensor_msgs.msg import Image, CameraInfo
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.grasping import Grasp2D, SuctionPoint2D, GraspAction

# Set up logger.
logger = Logger.get_logger("ros_nodes/test_grasp_planner.py")

def main():
    rospy.init_node("grasp_planning_example")
    logging.getLogger().addHandler(rl.RosStreamHandler())

    namespace = "gqcnn"
    vis_grasp = True

    cv_bridge = CvBridge()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.wait_for_service("%s/grasp_planner" % (namespace))
        plan_grasp = rospy.ServiceProxy("%s/grasp_planner" % (namespace),
                                        GQCNNGraspPlanner)

        color_im_msg = rospy.wait_for_message('camera/color/image_raw', Image)
        depth_im_msg = rospy.wait_for_message('camera/aligned_depth_to_color/image_raw', Image)
        camera_intr_msg = rospy.wait_for_message('camera/color/camera_info', CameraInfo)
        
        # DexNet expects float type
        depth_im_cv = cv_bridge.imgmsg_to_cv2(
                depth_im_msg, desired_encoding="passthrough").astype(np.float32)
        
        # Big/Little endian?
        #depth_im_cv = np.absolute(depth_im_cv-depth_im_cv.max())
        depth_im = cv_bridge.cv2_to_imgmsg(depth_im_cv, encoding='passthrough')

        grasp_resp = plan_grasp(color_im_msg, depth_im, camera_intr_msg)

        grasp = grasp_resp.grasp
        print(grasp_resp.grasp)
        print(grasp_resp.grasp.pose)

        camera_intr = CameraIntrinsics(
            frame=camera_intr_msg.header.frame_id,
            fx=camera_intr_msg.K[0],
            fy=camera_intr_msg.K[2],
            cx=camera_intr_msg.K[4],
            cy=camera_intr_msg.K[5],
            skew=camera_intr_msg.K[1],
            height=camera_intr_msg.height,
            width=camera_intr_msg.width
            )

        # Convert to a grasp action.
        grasp_type = GQCNNGrasp.SUCTION
        if grasp_type == GQCNNGrasp.PARALLEL_JAW:
            center = Point(np.array([grasp.center_px[0], grasp.center_px[1]]),
                        frame=camera_intr_msg.header.frame_id)
            grasp_2d = Grasp2D(center,
                            grasp.angle,
                            grasp.depth,
                            width=0.05,
                            camera_intr=camera_intr)
        elif grasp_type == GQCNNGrasp.SUCTION:
            print("suck")
            
            center = Point(np.array([grasp.center_px[0], grasp.center_px[1]]),
                        frame=camera_intr_msg.header.frame_id)
            grasp_2d = SuctionPoint2D(center,
                                    np.array([0, 0, 1]),
                                    grasp.depth,
                                    camera_intr=camera_intr)
        else:
            raise ValueError("Grasp type %d not recognized!" % (grasp_type))
        try:
            thumbnail = DepthImage(cv_bridge.imgmsg_to_cv2(
                grasp.thumbnail, desired_encoding="passthrough"),
                                frame=camera_intr_msg.header.frame_id)
        except CvBridgeError as e:
            logger.error(e)
            logger.error("Failed to convert image")
            sys.exit(1)
        action = GraspAction(grasp_2d, grasp.q_value, thumbnail)

        # Vis final grasp.
        depth_im_obj = DepthImage(depth_im_cv,
                                frame=depth_im.header.frame_id)
        color_im_obj = ColorImage(cv_bridge.imgmsg_to_cv2(color_im_msg, "rgb8"), 
                                frame=color_im_msg.header.frame_id)

        if vis_grasp:
            vis.figure(size=(10, 10))
            # vis.imshow(depth_im_obj, vmin=0.6, vmax=0.9)
            vis.imshow(color_im_obj, vmin=0.6, vmax=0.9)
            vis.grasp(action.grasp, scale=2.5, show_center=False, show_axis=True)
            vis.title("Planned grasp grasp at depth {0:.3f}m with Q={1:3f}".format(
            action.grasp.depth, action.q_value))
            vis.show()
        
        rate.sleep()

if __name__ == "__main__":
    main()