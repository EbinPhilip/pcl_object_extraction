import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge.core import CvBridge

from yolo_object_detection.srv import ObjectDetection
from yolo_object_detection.srv import ObjectDetectionResponse, ObjectDetectionRequest
from yolo_object_detection.msg import DetectedObjectArray, DetectedObject

from pcl_object_extraction.srv import ObjectExtraction, ObjectExtractionRequest, ObjectExtractionResponse

objectDetectorService = None
objectExtractorService = None

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
            return
    

if __name__ == '__main__':
    rospy.init_node('object_detection_client_node')

    rospy.Subscriber("/camera/color/image_raw", Image, imageCallback)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, cloudCallback)

    rospy.wait_for_service('object_detection_service')
    objectDetectorService = rospy.ServiceProxy('object_detection_service', ObjectDetection)

    rospy.wait_for_service('object_extraction_service')
    objectExtractorService = rospy.ServiceProxy('object_extraction_service', ObjectExtraction)
    
    pointCloudPublisher = rospy.Publisher('object_cloud', PointCloud2, queue_size=1)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        callback()
        rate.sleep()