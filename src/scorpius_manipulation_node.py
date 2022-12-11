import rospy
import tf
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import tf2_ros

import math

from geometry_msgs.msg import TransformStamped
from scorpius_manipulation.srv import SceneUpdate, SceneUpdateResponse, SceneUpdateRequest
from scorpius_manipulation.msg import SceneObject

import manipulator

def deg2rad(deg):
    return float((float(deg)/180.0) * math.pi)

placePose = TransformStamped()
placePose.header.frame_id = 'base_link'
placePose.child_frame_id = "place"
placePose.transform.translation.x = 0.23
placePose.transform.translation.y = -0.1
placePose.transform.translation.z = 0.23
quat = tf.transformations.quaternion_from_euler(
                0.0, deg2rad(90.0), 0.0)
placePose.transform.rotation.x = quat[0]
placePose.transform.rotation.y = quat[1]
placePose.transform.rotation.z = quat[2]
placePose.transform.rotation.w = quat[3]

if __name__ == "__main__":
    rospy.init_node('scorpius_manipulation', anonymous=False)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    placePose.header.stamp = rospy.Time.now()
    broadcaster.sendTransform(placePose)

    rospy.sleep(1)

    sceneUpdateService = rospy.ServiceProxy(
            'scene_update_service', SceneUpdate)
    arm = manipulator.Manipulator('scorpius_arm')

    arm.moveToPose("place")

    response:SceneUpdateResponse
    response = sceneUpdateService.call(SceneUpdateRequest())

    scene = PlanningSceneInterface()

    object: SceneObject
    for object in response.objects.objects:
        objectName = object.name
        arm.pickAndPlace(objectName, "place")
        rospy.sleep(0.1)
        scene.remove_attached_object(objectName)
        scene.remove_world_object(objectName)
        rospy.sleep(0.5)