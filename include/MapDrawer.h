/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include <pangolin/pangolin.h>
         
#include <mutex>

// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
// pcl
#include <pcl/io/pcd_io.h>// 读写
#include <pcl/common/transforms.h>// 点云坐标变换
#include <pcl/point_types.h>      // 点类型
#include <pcl/filters/voxel_grid.h>// 体素格滤波
#include <pcl/filters/passthrough.h>//  直通滤波
#include <pcl/sample_consensus/method_types.h>// 采样一致性，采样方法
#include <pcl/sample_consensus/model_types.h>// 模型
#include <pcl/segmentation/sac_segmentation.h>// 采样一致性分割
#include <pcl/filters/extract_indices.h>// 提取点晕索引

#include <octomap/ColorOcTree.h>

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);

    Map* mpMap;

    // 显示点======普通点黑色===参考地图点红色===颜色可修改====
    void DrawMapPoints();
    // 显示关键帧================蓝色====关键帧连线偏绿色
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    // 显示当前帧 相机位姿========绿色==
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    // 设置当前帧 相机姿==========
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    void SetReferenceKeyFrame(KeyFrame *pKF);// 没看到
    // 获取当前相机位姿，返回 OpenGlMatrix 类型=====
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    // 画线显示格子
    void DrawGrid();
    // 将高度转换成RGB颜色
    void heightMapColor(double h, double& r, double& g, double& b);
    // 画Octree地图
    void DrawOctoMap();
    // 保存地图
    void SaveOctoMap(const char *filename);
    // 载入地图
    void LoadOctoMap();
    // octoMap更新
    void UpdateOctomap(vector<KeyFrame*> vKFs);
    // 点云分割
    void GeneratePointCloud(KeyFrame *kf, 
                                    pcl::PointCloud<pcl::PointXYZRGB> &ground, 
                                    pcl::PointCloud<pcl::PointXYZRGB> &nonground);
    // 插入点云到octoMap
    void InsertScan(octomap::point3d sensorOrigin,
                                    pcl::PointCloud<pcl::PointXYZRGB> &ground,
                                    pcl::PointCloud<pcl::PointXYZRGB> &nonground);
private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

    /* ---------------------------------- */
    uint16_t  lastKeyframeSize =0;// 上一次关键帧数量大小
    octomap::ColorOcTree *m_octree;
    octomap::KeyRay m_keyRay; // temp storage for casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    bool m_useHeightMap;

    double m_colorFactor;
    double m_res;// octomap 图精度
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    bool bIsLocalization;

    octomap::OcTreeKey m_paddedMinKey, m_paddedMaxKey;
    inline static void updateMinKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& min)
    {
        for(unsigned int i=0; i<3; i++)
            min[i] = std::min(in[i], min[i]);
    }
    inline static void updateMaxKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& max)
    {
        for(unsigned int i=0; i<3; i++)
            max[i] = std::max(in[i], max[i]);
    }
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
