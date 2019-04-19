/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <condition_variable>

#include <pcl/octree/octree_search.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace ORB_SLAM2
{
class LoopClosing;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloudMapping(double resolution_, LoopClosing* loopCloser_);
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame(KeyFrame* kf);
    void shutdown();
    void viewer();
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf);
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& depth);

    pcl::PointCloud< PointCloudMapping::PointT>::Ptr convertToPointCloud(KeyFrame* kf);
    pcl::PointCloud<PointCloudMapping::PointT>::Ptr transformPointCloud(pcl::PointCloud<PointCloudMapping::PointT>& pointCloud, const Eigen::Isometry3d& t);

    PointCloud::Ptr globalMapRGBD;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalMapD;

    shared_ptr<thread>  viewerThread;   
    
    bool shutDownFlag = false;
    mutex shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize = 0;
    
    double resolution = 0.01;
    // point cloud xyzrgb
    pcl::VoxelGrid<PointT>  voxel;
    pcl::StatisticalOutlierRemoval<PointT> sor;// 创建滤波器对象
	pcl::RandomSample<PointT> *random_filter;

    // Resolution of voxel grid filter.
    double voxelResolution = 0.2;
    // Precentage of points to descard, must be between 0 - 1.0
    double decimatePercentage = 0.8;

    LoopClosing* loopCloser;
    std::vector<PointCloud> keyFrameCloud;
};
}
#endif // POINTCLOUDMAPPING_H
