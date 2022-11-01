#include <ros/ros.h>

#include <pcl_object_extraction/MeshCreation.h>
#include <pcl_object_extraction/MeshCreationRequest.h>
#include <pcl_object_extraction/MeshCreationResponse.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include "cloud_to_mesh.h"
#include "mesh_conversions.h"

using namespace vigir_point_cloud_proc;

ros::Publisher* markerPublisher;

bool createMesh(pcl_object_extraction::MeshCreation::Request& request,
        pcl_object_extraction::MeshCreation::Response& response)
{
    // convert request to pcl pointcloud
    pcl::PCLPointCloud2 inputPc2;
    pcl_conversions::toPCL(request.pointcloud, inputPc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(inputPc2, *inputCloud);

    // create mesh from pointcloud
    CloudToMesh cloudToMesh;
    pcl::PolygonMesh mesh;
    cloudToMesh.setInput(inputCloud);
    cloudToMesh.computeMesh();
    mesh = cloudToMesh.getMesh();

    // add mesh to response
    meshToShapeMsg(mesh, response.mesh);

    // publish visualization marker for the constructed mesh
    visualization_msgs::Marker mesh_marker;
    meshToMarkerMsg(mesh ,mesh_marker);
    markerPublisher->publish(mesh_marker);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_creation_service_node");
    ros::NodeHandle n;

    markerPublisher = new ros::Publisher(n.advertise<visualization_msgs::Marker>("/meshes", 1));

    ros::ServiceServer service = n.advertiseService("mesh_creation_service", createMesh);
    ros::spin();

    delete markerPublisher;
    return 0;
}
