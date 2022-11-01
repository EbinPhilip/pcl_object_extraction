#include "ObjectExtractor.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

ObjectBox ObjectExtractor::getObjectBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                        yolo_object_detection::BoundingBox box)
{
    int xmax = box.right;
    int xmin = box.left;
    int ymax = box.bottom;
    int ymin = box.top;

    int center_x, center_y;
    center_x = (xmax + xmin) / 2;
    center_y = (ymax + ymin) / 2;

    float maxx, minx, maxy, miny, maxz, minz;
    maxx = maxy = maxz = -std::numeric_limits<float>::max();
    minx = miny = minz = std::numeric_limits<float>::max();

    for (int i = xmin; i < xmax; i++)
    {
        for (int j = ymin; j < ymax; j++)
        {
            int pcl_index = (j * cloud->width) + i;
            pcl::PointXYZRGB point = cloud->at(pcl_index);

            if (std::isnan(point.x))
                continue;

            maxx = std::max(point.x, maxx);
            maxy = std::max(point.y, maxy);
            maxz = std::max(point.z, maxz);
            minx = std::min(point.x, minx);
            miny = std::min(point.y, miny);
            minz = std::min(point.z, minz);
        }
    }

    Eigen::Vector4f min(minx, miny, minz, 0);
    Eigen::Vector4f max(maxx, maxy, maxz, 0);
    ObjectBox objectBox(min, max);

    return objectBox;
}

void ObjectExtractor::extractBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
                                 ObjectBox box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)
{
    pcl::CropBox<pcl::PointXYZRGB> filter;
    filter.setMax(box.max);
    filter.setMin(box.min);
    filter.setInputCloud(inputCloud);

    filter.filter(*outputCloud);
}

void ObjectExtractor::removeGroundPlane(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)
{

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.002, FLT_MAX);
    // pass.setNegative (true);
    pass.filter(*outputCloud);
}

void ObjectExtractor::removeStatisticalOutlier(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (inputCloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (3.0);
    sor.filter (*outputCloud);
}
