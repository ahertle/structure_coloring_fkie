/*
 * StructurePublisher.h
 *
 *  Created on: 11 Feb 2013
 *      Author: andreas
 */

#ifndef STRUCTUREPUBLISHER_H_
#define STRUCTUREPUBLISHER_H_

#include <structureColoring/OcTree.h>
#include <structureColoring/NodePair.h>
#include <structureColoring/histograms/SphereUniformSampling.h>
#include <structureColoring/structures/PlanePatch.h>
#include <structureColoring/structures/CylinderPatch.h>
#include <structureColoring/NormalEstimationValueClass.h>
#include <octreelib/spatialaggregate/octree.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

class StructurePublisher
{
public:
    typedef PlanePatch::PlanePatches PlanePatches;
    typedef std::vector<PlanePatch::PlanePatchPtr> PlanePatchVector;
    typedef CylinderPatch::CylinderPatches CylinderPatches;
    typedef algorithm::OcTreeSamplingMap<float, NormalEstimationValueClass> OctreeSamplingMap;
    typedef std::vector<SphereUniformSampling> SphereUniformSamplings;
    typedef PlanePatch::PointIndices PointIndices;
    typedef Eigen::Vector3f Vec3;
    typedef PlanePatch::Points Points;
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef OcTree::NodePointers NodePointers;
    typedef OcTree::NodePointerList NodePointerList;

    StructurePublisher(ros::NodeHandle& nh);
    virtual ~StructurePublisher();

    template<typename PointAssignment, typename StructureContainer>
    void publishSegmentedPointcloud(const PointAssignment& segmented, const StructureContainer& planes, const PointCloud& pointCloud);

private:
    ros::Publisher mHistogramPC2Publisher;
    ros::Publisher mPC2Publisher;
    ros::Publisher mSegmentedCloudPublisher;
    ros::Publisher mCylinderPointCloudPublisher;
    ros::Publisher mMarkerPublisher;
    ros::Publisher mCylinderMarkerPublisher;
    std::string mFrameID;
    std::string mLaserTopic;
};

template<typename PointAssignment, typename StructureContainer>
void StructurePublisher::publishSegmentedPointcloud(const PointAssignment& segmented, const StructureContainer& planes, const PointCloud& pointCloud)
{
    if (mSegmentedCloudPublisher.getNumSubscribers() == 0)return;

    pcl::PointCloud<pcl::PointXYZRGB> segCloud;
    for (unsigned int i=0; i<segmented.size(); i++){

//      segCloud.points.push_back( pointCloud.points[i] );

        if (!segmented[i]){
            unsigned int rgb = 0x000000;
            pcl::PointXYZRGB p;
            p.x = pointCloud.points[i].x;
            p.y = pointCloud.points[i].y;
            p.z = pointCloud.points[i].z;
            p.rgb = *(float*)&rgb;
            segCloud.points.push_back(p);
        }
    }

    unsigned int i=0;
    for (typename StructureContainer::const_iterator pit = planes.begin(); pit != planes.end() ; pit++){
        float rgb[3]={0.f,0.2f,0.5f};
//        getColorByIndex(rgb, i, planes.size());
        unsigned int red = rgb[0] * 255.f + 0.5f;
        unsigned int green = rgb[1] * 255.f + 0.5f;
        unsigned int blue = rgb[2] * 255.f + 0.5f;
        unsigned int color = blue + (green<<8) + (red<<16);
        PointIndices inlierIndices = (*pit)->getInliers();
        for (PointIndices::iterator it=inlierIndices.begin(); it != inlierIndices.end(); it++){
            pcl::PointXYZRGB p;
            p.x = pointCloud.points[*it].x;
            p.y = pointCloud.points[*it].y;
            p.z = pointCloud.points[*it].z;
            p.rgb = *(float*)&color;
            segCloud.points.push_back(p);

        }
        i++;
    }

    sensor_msgs::PointCloud2 segMsg;
    pcl::toROSMsg<pcl::PointXYZRGB>(segCloud, segMsg);
    segMsg.header.frame_id=mFrameID;
    segMsg.header.stamp=ros::Time::now();
    mSegmentedCloudPublisher.publish(segMsg);
}

#endif /* STRUCTUREPUBLISHER_H_ */
