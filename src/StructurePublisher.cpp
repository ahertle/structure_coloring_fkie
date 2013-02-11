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
//    mHistogramPC2Publisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/histogramspc2", 5);
//    mPC2Publisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/pc2", 5);
    mSegmentedNormalCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/segmentednormalcloud2", 5);
//    mMarkerPublisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
//    mCylinderMarkerPublisher = nh.advertise<visualization_msgs::Marker>("cylinder_marker", 100);
//    mCylinderPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/cylinderPoints", 5);
    mFrameID = "/openni_rgb_optical_frame";
}

StructurePublisher::~StructurePublisher()
{
}

void StructurePublisher::getColorByIndex(float *rgb, unsigned int index, unsigned int total){
    float t = 1 - (static_cast<float>(index)/static_cast<float>(total));
// set rgb values
    if(t == 1.0){
        rgb[0] = 1.0f;
        rgb[1] = 0.0f;
        rgb[2] = 0.0f;
    } else if (t < 1.0 && t >= 0.83333) {
        rgb[0] = 1.0f;
        rgb[1] = 1.0f-(t-0.83333)/0.16666;
        rgb[2] = 0.0f;
    } else if (t < 0.83333 && t >= 0.66666) {
        rgb[0] = (t-0.66666)/0.16666;
        rgb[1] = 1.0f;
        rgb[2] = 0.0f;
    } else if (t < 0.66666 && t >= 0.5) {
        rgb[0] = 0.0f;
        rgb[1] = 1.0f;
        rgb[2] = 1.0f-(t-0.5)/0.16666;
    } else if (t < 0.5 && t >= 0.33333){
        rgb[0] = 0.0f;
        rgb[1] = 1.0f-(t-0.33333)/0.16666;
        rgb[2] = 1.0f;
    } else if (t < 0.33333 && t >= 0.16666){
        rgb[0] = (t-0.16666)/0.16666;
        rgb[1] = 0.0f;
        rgb[2] = 1.0f;
    } else {
        rgb[0] = 1.0f;
        rgb[1] = 0.0f;
        rgb[2] = 1.0f-t/0.16666;
    }
}

