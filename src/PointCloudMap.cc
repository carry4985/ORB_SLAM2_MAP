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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <time.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include "PointCloudMap.h"
#include "KeyFrame.h"
#include "Converter.h"

namespace ORB_SLAM2
{
PointCloudMapping::PointCloudMapping(double resolution_, LoopClosing* loopCloser_)
{
    this->resolution = resolution_;

    // down_simple point cloud
    voxel.setLeafSize(resolution, resolution, resolution);
    random_filter = new pcl::RandomSample<PointT>;


    // remove the outlier point
    this->sor.setMeanK(50);                                //设置在进行统计时考虑查询点邻近点数
    this->sor.setStddevMulThresh(1.0);                    //设置判断是否为离群点的阈值

    globalMapRGBD = boost::make_shared<PointCloud>();
    globalMapD = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    // use thread for point cloud viewer
    viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));

    // copy closer
    loopCloser = loopCloser_;
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back(kf);
   
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf)
{
    PointCloud::Ptr tmp(new PointCloud());
    // point cloud is null ptr
    for ( int m=0; m<kf->mImDep.rows; m+=3 )
    {
        for ( int n=0; n<kf->mImDep.cols; n+=3 )
        {
            float d = kf->mImDep.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;
            
            p.b = kf->mImRGB.ptr<uchar>(m)[n*3];
            p.g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
            p.r = kf->mImRGB.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }

    // 相机到世界坐标系的坐标变换 Tcw
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    
    // std::cout << "generate point cloud for kf： " << kf->mnId << ", size= " << cloud->points.size() << std::endl;
    return cloud;
}

pcl::PointCloud< PointCloudMapping::PointT>::Ptr PointCloudMapping::convertToPointCloud(KeyFrame* kf)
{
    PointCloud::Ptr tmp(new PointCloud);

    for ( int m=0; m<kf->mImDep.rows; m+=3 )
    {
        for ( int n=0; n<kf->mImDep.cols; n+=3 )
        {
            float d = kf->mImDep.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;
            
            p.b = kf->mImRGB.ptr<uchar>(m)[n*3];
            p.g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
            p.r = kf->mImRGB.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }

    tmp->is_dense = false;
    return tmp;
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::transformPointCloud(pcl::PointCloud<PointCloudMapping::PointT>& pointCloud, const Eigen::Isometry3d& t)
{
    // 本身就是智能指针，不用考虑内存泄露
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud(pointCloud, *cloud, t.inverse().matrix());
    cloud->is_dense = false;

    return cloud;
} 

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& depth)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            pcl::PointXYZ p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
                
            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    
    // cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

#define POINTRGB

#ifdef POINTRGB
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            // do nothing until we have a new keyframe
            unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = keyframes.size();
        }
        
        for (size_t i=lastKeyframeSize; i<N ; i++)
        {
            PointCloud::Ptr p;
            // just convert to pointcloud, no transform
            p = convertToPointCloud(keyframes[i]);
            // p = keyframes[i]->keyFramePointCloud;
            // save the pointcloud
            keyFrameCloud.push_back(*p);
        }

        if(loopCloser->loop_detected)
        {
            loopCloser->loop_detected = false;
            std::cout << "detect a loop closure" << std::endl;
            std::vector<ORB_SLAM2::KeyFrame*> key_frames = loopCloser->getMap()->GetAllKeyFrames();
            // 根据ID对关键帧排序
            sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);

            // reset the map data, then regenerate
            globalMapRGBD.reset(new PointCloud);
            // generate the global map
            for(auto key_frame : key_frames){
                if(key_frame->isBad())
                    continue;
            
                PointCloud::Ptr p;
                p = generatePointCloud(key_frame);
                // p = key_frame->keyFramePointCloud;
                // 两组点云合成， PointCloud已经重载了 += 操作符
                *globalMapRGBD += *p;
            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
            voxel.setInputCloud(globalMapRGBD);
            voxel.filter(*tmp);
    
            globalMapRGBD->swap(*tmp);
        }else{
            PointCloud::Ptr p;
            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(keyframes[lastKeyframeSize]->GetPose());
            p = transformPointCloud(keyFrameCloud[keyFrameCloud.size() - 1], T);
            // 两组点云合成， PointCloud已经重载了 += 操作符
            *globalMapRGBD += *p;
            // temp variable for filter
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

            // const int n_points = static_cast<int>((1.0 - decimatePercentage) *
            //                                     globalMapRGBD->size());
            // random_filter->setSample(n_points);
            // random_filter->setInputCloud(globalMapRGBD);
            // random_filter->filter(*tmp);

            voxel.setInputCloud(globalMapRGBD);
            voxel.filter(*tmp);
    
            globalMapRGBD->swap(*tmp);
            viewer.showCloud(globalMapRGBD); // show the pointcloud without optimization
            cout<<"show global map, size="<<globalMapRGBD->points.size() << " " << N <<endl;
            lastKeyframeSize = N;
        }
    
    }

    globalMapRGBD->clear();
    // save the downsample pointcloud
    for(size_t i=0;i<keyframes.size();i++)
    {
        cout<<"keyframe "<<i<<" ..."<<endl;
        PointCloud::Ptr tp = generatePointCloud(keyframes[i]);
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud(tp);
        voxel.filter( *tmp );
        // 在PointCloud实现中，我们同样是实现循环拷贝
        *globalMapRGBD += *tmp;
        viewer.showCloud(globalMapRGBD);
    }
    PointCloud::Ptr tmp(new PointCloud());
    sor.setInputCloud(globalMapRGBD);
    sor.filter(*tmp);
    globalMapRGBD->swap(*tmp);         
    pcl::io::savePCDFileBinary ("optimized_pointcloud.pcd", *globalMapRGBD);
    cout<<"Save point cloud file successfully!"<<endl;
}

#else
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = keyframes.size();
        }
        
        for (size_t i=lastKeyframeSize; i<N; i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr p = generatePointCloud(keyframes[i], depthImgs[i]);
            // 两组电云合成， PointCloud已经重载了 += 操作符
            *globalMapD += *p;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(0.05);
        octree.setInputCloud(globalMapD);
        octree.addPointsFromInputCloud();
        octree.getOccupiedVoxelCenters(tmp->points);

        globalMapD->swap(*tmp);
        viewer.showCloud(globalMapD);                                        // show the pointcloud without optimization
        cout<<"show global map, size=" << globalMapD->points.size() << " " << N <<endl;
        lastKeyframeSize = N;
    }
}

#endif
} // namespace ORB_SLAM2