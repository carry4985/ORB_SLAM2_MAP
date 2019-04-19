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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>


namespace ORB_SLAM2
{

class Map;       // 地图
class MapPoint;  // 地图点
class Frame;     // 普通帧
class KeyFrameDatabase; // 关键帧数据库 存储关键点 位姿态等信息 用于匹配

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB,
            cv::Mat& rgb, cv::Mat& depth);
    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();// 位姿
    cv::Mat GetPoseInverse();// 位姿
    cv::Mat GetCameraCenter();// 单目 相机中心
    cv::Mat GetStereoCenter();// 双目 相机中心
    cv::Mat GetRotation();// 旋转矩阵
    cv::Mat GetTranslation();// 平移向量

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    // 可视化 添加 线连接  关键点连线
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);// 删除 线连接
    void UpdateConnections();// 跟新线连接
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions 生成 关键帧 树 
    void AddChild(KeyFrame* pKF);// 添加孩子
    void EraseChild(KeyFrame* pKF);// 删除孩子
    void ChangeParent(KeyFrame* pKF);// 跟换父亲
    std::set<KeyFrame*> GetChilds();// 得到孩子
    KeyFrame* GetParent();//得到父亲
    bool hasChild(KeyFrame* pKF);// 有孩子吗

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);  // 添加闭环边
    std::set<KeyFrame*> GetLoopEdges(); 

    // MapPoint observation functions
    // 地图点 观测函数
    void AddMapPoint(MapPoint* pMP, const size_t &idx);// 添加 地图点
    void EraseMapPointMatch(const size_t &idx);// 删除地图点匹配
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);// 替换地图点匹配
    std::set<MapPoint*> GetMapPoints();// 得到地图点集合 set 指针
    std::vector<MapPoint*> GetMapPointMatches();// 得到地图点匹配
    int TrackedMapPoints(const int &minObs);// 跟踪到的地图点
    MapPoint* GetMapPoint(const size_t &idx);//得到单个地图点

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    cv::Mat mImRGB;// 彩色图
    cv::Mat mImDep;// 深度图
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keyFramePointCloud;
    
    void convertToPointCloud(void);
    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;  // 上一个关键帧帧的Id
    long unsigned int mnId;            // 当前关键帧的Id
    const long unsigned int mnFrameId; // 图像帧的Id

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    // 用于图像分割，加速特征匹配
    const int mnGridCols; // 分割后图像的列数
    const int mnGridRows; // 分割后图像的行数
    const float mfGridElementWidthInv; // 分割后每个像素占用的格子数量
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF; // 做过相邻匹配 标志 在 LocalMapping::SearchInNeighbors() 使用

    // Variables used by the local mapping
    // 本地地图  最小化重投影误差 BA参数
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    // 关键帧数据库 变量参数
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    // 校准参数 相机内参 基线*焦距 基线 深度
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    // 当前关键帧 提取到体征点的数量
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;// 世界 到 相机
    cv::Mat Twc;// 相机 到 世界
    cv::Mat Ow;//  相机坐标

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints; // 地图点

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
