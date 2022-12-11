#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from scene_objects_finder import SceneObjectsFinder
from scene_updater import SceneUpdater
from grasp_calculator import GraspCalculator
from scorpius_manipulation.msg import SceneObject, SceneObjectArray
from scorpius_manipulation.srv import SceneUpdate, SceneUpdateRequest, SceneUpdateResponse

import tf2_ros

image = None
cloud = None
sceneObjectsFinder = None
sceneUpdater = None
graspCalculator = None
broadcaster = tf2_ros.StaticTransformBroadcaster()

objectDetectionServicePath = 'object_detection_service'
objectExtractionServicePath = 'object_extraction_service'
meshCreationServicePath = 'mesh_creation_service'


def imageCallback(img:Image):
    global image
    image = img

def cloudCallback(cloud2:PointCloud2):
    global cloud
    cloud = cloud2

def updateScene(request:SceneUpdateRequest)->SceneUpdateResponse:
    if image==None:
        print("image not initialized")
        return
    if cloud==None:
        print("cloud not initialized")
        return

    sceneObjectArray = sceneObjectsFinder.getSceneObjects(image, cloud)
    sceneUpdater.updateScene(sceneObjectArray, 'base_link')

    sceneObject:SceneObject
    for sceneObject in sceneObjectArray.objects:
        tfMessage = graspCalculator.getGrasp(sceneObject.boundingBox,
                parentFrame='base_link', childFrameName=sceneObject.name)
        broadcaster.sendTransform(tfMessage)

    sceneUpdateResponse = SceneUpdateResponse()
    sceneUpdateResponse.objects = sceneObjectArray
    return sceneUpdateResponse

if __name__ == '__main__':
    rospy.init_node('mesh_creation_client_node')

    rospy.Subscriber("/camera/color/image_raw", Image, imageCallback)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, cloudCallback)
    rospy.wait_for_message("/camera/color/image_raw", Image)
    rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)

    rospy.wait_for_service(objectDetectionServicePath)
    rospy.wait_for_service(objectExtractionServicePath)
    rospy.wait_for_service(meshCreationServicePath)

    sceneObjectsFinder = SceneObjectsFinder(objectDetectionServicePath,
            objectExtractionServicePath, meshCreationServicePath)
    sceneUpdater = SceneUpdater()
    graspCalculator = GraspCalculator()
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    service = rospy.Service('scene_update_service', SceneUpdate, updateScene)

    rospy.spin()