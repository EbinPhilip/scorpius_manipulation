from scorpius_manipulation.msg import SceneObject, SceneObjectArray
from sensor_msgs.msg import PointCloud2, Image
from yolo_object_detection.msg import DetectedObject, DetectedObjectArray
from yolo_object_detection.srv import ObjectDetection, ObjectDetectionRequest, ObjectDetectionResponse
from pcl_object_extraction.srv import ObjectExtraction, ObjectExtractionRequest, ObjectExtractionResponse
from pcl_object_extraction.srv import MeshCreation, MeshCreationRequest, MeshCreationResponse

import copy

import rospy


class SceneObjectsFinder:
    def __init__(self,
                 objectDetectionServicePath: str = 'object_detection_service',
                 objectExtractionServicePath: str = 'object_extraction_service',
                 meshCreationServicePath: str = 'mesh_creation_service',
                 interestedObjects: list = ['apple', 'banana', 'bottle', 'cup']):
        self.objectDetectionService = rospy.ServiceProxy(
            objectDetectionServicePath, ObjectDetection)
        self.objectExtractionService = rospy.ServiceProxy(
            objectExtractionServicePath, ObjectExtraction)
        self.meshCreationService = rospy.ServiceProxy(
            meshCreationServicePath, MeshCreation)
        self.interestedObjects = interestedObjects

    def getSceneObjects(self, imageInput: Image, cloudInput: PointCloud2) -> SceneObjectArray:
        image = copy.copy(imageInput)
        cloud = copy.copy(cloudInput)

        detectRequest = ObjectDetectionRequest()
        detectRequest.image = image
        detectResponse: ObjectDetectionResponse
        detectResponse = self.objectDetectionService(detectRequest)

        sceneObjects = SceneObjectArray()
        objectsDict = dict()
        object:DetectedObject
        for object in detectResponse.detectedObjects.box:
            box = object.box
            if object.name not in self.interestedObjects:
                continue
            extractRequest = ObjectExtractionRequest()
            extractRequest.inputCloud = cloud
            extractRequest.boundingBox = box
            extractResponse:ObjectExtractionResponse
            extractResponse = self.objectExtractionService(extractRequest)

            meshCreationRequest = MeshCreationRequest()
            meshCreationRequest.pointcloud = extractResponse.extractedObject
            meshCreationResponse:MeshCreationResponse
            meshCreationResponse = self.meshCreationService(
                meshCreationRequest)

            sceneObject = SceneObject()
            if object.name in objectsDict:
                objectsDict[object.name] = objectsDict[object.name]+1
            else:
                objectsDict[object.name] = 1
            sceneObject.name = object.name + str(objectsDict[object.name])
            sceneObject.type = object.name
            sceneObject.cloud = extractResponse.extractedObject
            sceneObject.mesh = meshCreationResponse.mesh
            sceneObject.boundingBox = extractResponse.boundingBox3d

            sceneObjects.objects.append(sceneObject)

        return sceneObjects
