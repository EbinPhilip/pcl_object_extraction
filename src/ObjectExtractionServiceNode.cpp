#include <ros/ros.h>

#include <pcl_object_extraction/ObjectExtraction.h>
#include <pcl_object_extraction/ObjectExtractionRequest.h>
#include <pcl_object_extraction/ObjectExtractionResponse.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include "ObjectExtractor.h"

tf::TransformListener *tfListener;

bool extractObjects(pcl_object_extraction::ObjectExtraction::Request &request,
                    pcl_object_extraction::ObjectExtraction::Response &response)
{
    ObjectExtractor objectExtractor;

    // get pcl::pointcloud from ros pointcloud2 msg
    pcl::PCLPointCloud2 inputPc2;
    pcl_conversions::toPCL(request.inputCloud, inputPc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(inputPc2, *inputCloud);

    // extract objects using bounding boxes. This requires ordered cloud in camera frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    ObjectBox objectBox = objectExtractor.getObjectBox(inputCloud, request.boundingBox);
    objectExtractor.extractBox(inputCloud, objectBox, extractedCloud);

    // transform the extracted object to base_link frame
    // first convert back to ros pointcloud2 format, so as to use tf for transformation
    sensor_msgs::PointCloud2 extractedCloudMsg, extractedCloudTransformed;
    pcl::PCLPointCloud2 extractedCloudPc2;
    pcl::toPCLPointCloud2(*extractedCloud, extractedCloudPc2);
    pcl_conversions::fromPCL(extractedCloudPc2, extractedCloudMsg);
    extractedCloudMsg.header.stamp = ros::Time(0);
    pcl_ros::transformPointCloud("base_link", extractedCloudMsg, extractedCloudTransformed, *tfListener);

    // once again, get the pcl representation of the transformed object cloud
    pcl_conversions::toPCL(extractedCloudTransformed, extractedCloudPc2);
    extractedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(extractedCloudPc2, *extractedCloud);

    // remove ground plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    objectExtractor.removeGroundPlane(extractedCloud, groundFilteredCloud);

    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractedObject(new pcl::PointCloud<pcl::PointXYZRGB>());
    objectExtractor.removeStatisticalOutlier(groundFilteredCloud, extractedObject);

    response.boundingBox3d = objectExtractor.getBoundingBox(extractedObject);

    // convert back from pcl::pointcloud to ros pointcloud2 msg
    pcl::PCLPointCloud2 extractedObjectPc2;
    pcl::toPCLPointCloud2(*extractedObject, extractedObjectPc2);
    pcl_conversions::fromPCL(extractedObjectPc2, response.extractedObject);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_extraction_service_node");
    ros::NodeHandle n;

    tfListener = new tf::TransformListener();

    ros::ServiceServer service = n.advertiseService("object_extraction_service", extractObjects);
    ros::spin();

    delete tfListener;
    return 0;
}