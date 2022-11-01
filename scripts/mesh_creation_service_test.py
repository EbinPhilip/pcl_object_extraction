import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge.core import CvBridge

from yolo_object_detection.srv import ObjectDetection
from yolo_object_detection.srv import ObjectDetectionResponse, ObjectDetectionRequest
from yolo_object_detection.msg import DetectedObjectArray, DetectedObject

from pcl_object_extraction.srv import ObjectExtraction, ObjectExtractionRequest, ObjectExtractionResponse
from pcl_object_extraction.srv import MeshCreation, MeshCreationRequest, MeshCreationResponse

objectDetectorService = None
objectExtractorService = None
meshCreationServvice = None

pointCloudPublisher = None

image = None
cloud = None

def imageCallback(img:Image):
    global image
    image = img

def cloudCallback(cloud2:PointCloud2):
    global cloud
    cloud = cloud2

def callback():
    detectRequest = ObjectDetectionRequest()
    if image==None:
        print("image not initialized")
        return
    if cloud==None:
        print("cloud not initialized")
        return
    detectRequest.image = image
    cld = cloud
    detectResponse = objectDetectorService(detectRequest)
    for object in detectResponse.detectedObjects.box:
        if object.name == 'cup':
            box = object.box
            extractRequest = ObjectExtractionRequest()
            extractRequest.inputCloud = cld
            extractRequest.boundingBox = box
            extractResponse = objectExtractorService(extractRequest)
            global pointCloudPublisher
            pointCloudPublisher.publish(extractResponse.extractedObject)
            meshCreationRequest = MeshCreationRequest()
            meshCreationRequest.pointcloud = extractResponse.extractedObject
            meshCreationServvice(meshCreationRequest)
            return
    

if __name__ == '__main__':
    rospy.init_node('mesh_creation_client_node')

    rospy.Subscriber("/camera/color/image_raw", Image, imageCallback)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, cloudCallback)
    rospy.wait_for_message("/camera/color/image_raw", Image)
    rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)

    rospy.wait_for_service('object_detection_service')
    objectDetectorService = rospy.ServiceProxy('object_detection_service', ObjectDetection)

    rospy.wait_for_service('object_extraction_service')
    objectExtractorService = rospy.ServiceProxy('object_extraction_service', ObjectExtraction)

    rospy.wait_for_service('mesh_creation_service')
    meshCreationServvice = rospy.ServiceProxy('mesh_creation_service', MeshCreation)

    pointCloudPublisher = rospy.Publisher('object_cloud', PointCloud2, queue_size=1)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        callback()
        rate.sleep()