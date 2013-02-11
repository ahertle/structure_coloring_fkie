/*
 * StructurePublisher.cpp
 *
 *  Created on: 11 Feb 2013
 *      Author: andreas
 */

#include "structureColoring/StructurePublisher.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>

StructurePublisher::StructurePublisher(ros::NodeHandle& nh)
{
    mHistogramPC2Publisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/histogramspc2", 5);
    mPC2Publisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/pc2", 5);
    mSegmentedCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/segmentedcloud2", 5);
    mMarkerPublisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    mCylinderMarkerPublisher = nh.advertise<visualization_msgs::Marker>("cylinder_marker", 100);
    mCylinderPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/cylinderPoints", 5);
    mFrameID = "/openni_rgb_optical_frame";
}

StructurePublisher::~StructurePublisher()
{
}

