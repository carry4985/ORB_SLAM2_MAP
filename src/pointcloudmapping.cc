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

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;

    // down_simple point cloud
    voxel.setLeafSize(resolution, resolution, resolution);

    // remove the outlier point
    this->sor.setMeanK(50);                                //设置在进行统计时考虑查询点邻近点数
    this->sor.setStddevMulThresh(1.0);                    //设置判断是否为离群点的阈值

    // down_simple point cloud
    voxelD.setLeafSize(resolution, resolution, resolution);

    // remove the outlier point
    this->sorD.setMeanK(50);                                //设置在进行统计时考虑查询点邻近点数
    this->sorD.setStddevMulThresh(1.0);                    //设置判断是否为离群点的阈值

    globalMapRGBD = boost::make_shared<PointCloud>();
    globalMapD = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    // use thread for point cloud viewer
    viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
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

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
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
    
    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
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
            unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck(keyframeMutex);
            N = keyframes.size();
        }
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
            // 两组电云合成， PointCloud已经重载了 += 操作符
            *globalMapRGBD += *p;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (globalMapRGBD);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-2.5, -0.01);
        pass.filter (*tmp);
  
        // pcl::octree::OctreePointCloud<pcl::PointXYZRGBA> octree(0.05);
        // octree.setInputCloud(globalMapRGBD);
        // octree.addPointsFromInputCloud();
        // octree.getOccupiedVoxelCenters(tmp->points);

        // PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud(tmp);
        voxel.filter(*globalMapRGBD);
  
        // globalMapRGBD->swap(*tmp);
        viewer.showCloud(globalMapRGBD);                                        // show the pointcloud without optimization
        cout<<"show global map, size="<<globalMapRGBD->points.size() << " " << N <<endl;
        lastKeyframeSize = N;
    }
 //    if(!globalMapRGBD->empty())                                           // save the pointcloud without optimization
 //    {
 //    	PointCloud::Ptr tmp(new PointCloud());
	// sor.setInputCloud(globalMapRGBD);            // remove outlier points      
	// sor.filter( *tmp );
 //               globalMapRGBD->swap( *tmp );                    
 //    	pcl::io::savePCDFileBinary( "pointcloud.pcd", *globalMapRGBD );
 //    	cout<<"Save point cloud file successfully!"<<endl;
 //    }
 //    cout << endl <<"Start to show optimized map!"<<endl;

    globalMapRGBD->clear();
    for(size_t i=0;i<keyframes.size();i++)                               // save the optimized pointcloud
    {
        cout<<"keyframe "<<i<<" ..."<<endl;
        PointCloud::Ptr tp = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud(tp);
        voxel.filter( *tmp );
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
