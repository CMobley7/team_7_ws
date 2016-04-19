#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import rospy
import tf
from tf import TransformListener
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray


def intrinsic_callback(data):
    global intrinsicSubscriber
    global K
    tf = TransformListener()
    K = np.array(data.K).reshape(3,3)
    K = np.hstack((K,[[0],[0],[0]]))
    ## After acquiring the matrix unsubscribe from the topic (since the matrix is static)
    intrinsicSubscriber.unregister()


def projection_callback(data):
    poses = data.poses
    poseNumber = len(poses)
    poseMatrix = np.array([[0], [0], [1]])
    try:
        if poseNumber == 0:
            # rospy.loginfo('no object received')
            return
        for i in range(poseNumber):
            poseMatrix = np.append(poseMatrix, np.array([[poses[i].position.x],[poses[i].position.y], [1]]),1)

        poseMatrix = np.delete(poseMatrix, 0, 1)

        (trans,rot) = listener.lookupTransform('world', 'uav1/downward_cam_optical_frame', rospy.Time())
        # Construct transformation matrix with rotation and translation
        # Rotation first
        T = quaternion_matrix(rot)
        # Add translation vector into its place
        T[0,3] = trans[0];
        T[1,3] = trans[1];
        T[2,3] = trans[2];
        # Delete the 3rd col in the transformation matrix
        ## Enforcing "Z" to be 0
        T_star = np.delete(T, 2, 1)
        # Construct projection matrix with extrinsic and intrinsic matrices
        P_star = np.dot(K, T_star)
        P_star_inv = inv(P_star)
        point3D = np.dot(P_star_inv, poseMatrix)
        points3D = point3D/point3D[2]
        points3D_ = data

        for i in range(poseNumber):
            points3D_.poses[i].position.x = point3D[0, i]
            points3D_.poses[i].position.y = point3D[1, i]

        object_publisher.publish(points3D_)
        # point3D_.x, point3D_.y, point3D_.z = point3D[1], point3D[0], point3D[2]
        # object_publisher.publish(point3D_)
        # translationVector = np.array([[data.pose.position.x], [data.pose.position.y], [data.pose.position.z], [1]])
        # T = quaternion_matrix((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
        # T[0,3] = data.pose.position.x;
        # T[1,3] = data.pose.position.y;
        # T[2,3] = data.pose.position.z;
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "exception caught"


if __name__ == "__main__":
    rospy.init_node('object_localization')
    intrinsicSubscriber = rospy.Subscriber('/uav1/downward_cam/camera/camera_info', CameraInfo, intrinsic_callback)
    rospy.Subscriber('detected_target', PoseArray, projection_callback)
    object_publisher = rospy.Publisher('target_3d', PoseArray, queue_size=10)
    listener = tf.TransformListener()
    K = np.zeros((3,3), float)
    rospy.spin()
