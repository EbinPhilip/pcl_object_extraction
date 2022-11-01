#ifndef __OBJECT_EXTRACTOR_H__
#define __OBJECT_EXTRACTOR_H__

#include <yolo_object_detection/BoundingBox.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ObjectBox
{
    ObjectBox(Eigen::Vector4f min_, Eigen::Vector4f max_)
        : min(min_),
          max(max_)
    {
    }

    Eigen::Vector4f min;
    Eigen::Vector4f max;
};

class ObjectExtractor
{
    public:
    ObjectBox getObjectBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
            yolo_object_detection::BoundingBox box);

    void extractBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
            ObjectBox box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);

    void removeGroundPlane(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);

    void removeStatisticalOutlier(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);
};

#endif