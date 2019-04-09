/* 跟踪线程 深度 双目初始化位姿 运动模型 关键帧模式 重定位 局部地图跟踪 关键帧
*This file is part of ORB-SLAM2.
* 
* mpMap就是我们整个位姿与地图（可以想象成ORB-SLAM运行时的那个界面世界），
* MapPoint和KeyFrame都被包含在这个mpMap中。
* 因此创建这三者对象（地图，地图点，关键帧）时，
* 三者之间的关系在构造函数中不能缺少。
* 
* 另外，由于一个关键帧提取出的特征点对应一个地图点集，
* 因此需要记下每个地图点的在该帧中的编号；
* 
* 同理，一个地图点会被多帧关键帧观测到，
* 也需要几下每个关键帧在该点中的编号。
* 
* 地图点，还需要完成两个运算，第一个是在观测到该地图点的多个特征点中（对应多个关键帧），
* 挑选出区分度最高的描述子，作为这个地图点的描述子；
* pNewMP->ComputeDistinctiveDescriptors();
* 
* 第二个是更新该地图点平均观测方向与观测距离的范围，这些都是为了后面做描述子融合做准备。
pNewMP->UpdateNormalAndDepth();

* 
* 跟踪
* 每一帧图像 Frame ---> 提取ORB关键点特征 -----> 根据上一帧进行位置估计计算R t (或者通过全局重定位初始化位置)
* ------> 跟踪局部地图，优化位姿 -------> 是否加入 关键帧
* 
* Tracking线程
* 帧 Frame
* 1】初始化
*       单目初始化 MonocularInitialization()
*       双目初始化 StereoInitialization
* 
* 2】相机位姿跟踪P
*       同时跟踪和定位 同时跟踪与定位，不插入关键帧，局部建图 不工作
*       跟踪和定位分离 mbOnlyTracking(false)  
        位姿跟踪 TrackWithMotionModel()  TrackReferenceKeyFrame()  重定位 Relocalization()
*   
 a 运动模型（Tracking with motion model）跟踪   速率较快  假设物体处于匀速运动
      用 上一帧的位姿和速度来估计当前帧的位姿使用的函数为TrackWithMotionModel()。
      这里匹配是通过投影来与上一帧看到的地图点匹配，使用的是
      matcher.SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, ...)。
      
 b 关键帧模式      TrackReferenceKeyFrame()
     当使用运动模式匹配到的特征点数较少时，就会选用关键帧模式。即尝试和最近一个关键帧去做匹配。
     为了快速匹配，本文利用了bag of words（BoW）来加速。
     首先，计算当前帧的BoW，并设定初始位姿为上一帧的位姿；
     其次，根据位姿和BoW词典来寻找特征匹配，使用函数matcher.SearchByBoW(KeyFrame *pKF, Frame &F, ...)；
     匹配到的是参考关键帧中的地图点。
     最后，利用匹配的特征优化位姿。
     
c 通过全局重定位来初始化位姿估计 Relocalization() 
    假如使用上面的方法，当前帧与最近邻关键帧的匹配也失败了，
    那么意味着需要重新定位才能继续跟踪。
    重定位的入口如下： bOK = Relocalization();
    此时，只有去和所有关键帧匹配，看能否找到合适的位置。
    首先，计算当前帧的BOW向量，在关键帧词典数据库中选取若干关键帧作为候选。
         使用函数如下：vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    其次，寻找有足够多的特征点匹配的关键帧；最后，利用RANSAC迭代，然后使用PnP算法求解位姿。这一部分也在Tracking::Relocalization() 里

    
 * 3】局部地图跟踪
*       更新局部地图 UpdateLocalMap() 更新关键帧和 更新地图点  UpdateLocalKeyFrames()   UpdateLocalPoints
*       搜索地图点  获得局部地图与当前帧的匹配
*       优化位姿    最小化重投影误差  3D点-2D点对  si * pi = K * T * Pi = K * exp(f) * Pi 
* 
* 4】是否生成关键帧
*       加入的条件：
*       很长时间没有插入关键帧
*       局部地图空闲
*       跟踪快要跟丢
*       跟踪地图 的 MapPoints 地图点 比例比较少
* 
* 5】生成关键帧
*       KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB)
*       对于双目 或 RGBD摄像头构造一些 MapPoints，为MapPoints添加属性
* 
* 进入LocalMapping线程
   设置闭环检测帧  mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame); // ===
* 
* 


 void Tracking::CreateNewKeyFrame() 中向点云 建图 线程 添加关键帧======

*/


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
         
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"
         
#include "Optimizer.h"
#include "PnPsolver.h"

#include "pointcloudmapping.h"

#include <iostream>

#include <mutex>


using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyPoint数据类型

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, 
                            shared_ptr<PointCloudMapping> pPointCloud, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), 
    mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
    mpPointCloudMapping(pPointCloud)
{

    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    // 关键帧间隔
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // 创建ORB特征提取对象
    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(
                                        nFeatures,    // 提取特征点的总数: 1000
                                        fScaleFactor, // 图像金字塔尺度因子: 1.2
                                        nLevels,      // 金字塔层数: 8
                                        fIniThFAST,   // 快速角点提取参数 阈值
                                        fMinThFAST    // 快速角点提取参数 最小阈值: 8
                                        );

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        // 判断一个3D点远/近的阈值 mbf * 35 / fx  不可靠阈值
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {   
        // 深度图缩放尺度 rawDepth * mDepthMapFactor = 缩放后的深度
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(mDepthMapFactor==0)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

// 设置局部建图  system.cc 函数传递过来的 建图线程指针
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

// 设置回环检测  system.cc 函数传递过来的 回环检测线程指针
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

// 设置 可视化
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    // 1. 无论图片是RGB，BGR， 还是RGBA，BGRA，均转化为灰度图，放弃彩色信息
    mImRGB = imRGB;
    mImGray = imRGB;
    mImDepth = imD;

    // 2. 图像转换成灰度图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // 3. 深度相机深度图 精度转换
    if(mDepthMapFactor!=1 || mImDepth.type()!=CV_32F)
    {
        mImDepth.convertTo(mImDepth, CV_32F, mDepthMapFactor);
    }

    // 2. 进行特征检测
    // 代码ORB_SLAM2_SSD_Semantic在此处加入基于光流的动态物体检测代码
    mCurrentFrame = Frame(mImGray, mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // 3. 运动跟踪(跟踪上一帧 地图点)/参考帧跟踪(跟踪上一参考关键帧地图点)/重定位(跟踪所有关键帧地图点) 得到位姿
    // 局部地图点跟踪 再优化位姿
    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

/***************************************************************
// 跟踪关键点 计算 相机位姿
追踪这部分主要用了几种模型：
运动模型 跟踪（Tracking with motion model）、
参考关键帧 跟踪（Tracking with reference key frame）和
重定位（Relocalization） 跟踪。

【1】运动模型 跟踪（Tracking with motion model）
    跟踪上一帧的地图点
    
    上一帧的地图点 反投影到当前帧图像像素坐标上  和 当前帧的 关键点落在 同一个 格子内的 
    做描述子匹配 搜索 可以加快匹配
        
	假设物体处于匀速运动，那么可以用上一帧的位姿和速度来估计当前帧的位姿。
	上一帧的速度可以通过前面几帧的位姿计算得到。
	这个模型适用于运动速度和方向比较一致，没有大转动的情形下，比如匀速运动的汽车、机器人、人等。
	而对于运动比较随意的目标，当然就会失效了。此时就要用到下面两个模型。

【2】参考关键帧 跟踪（Tracking with reference key frame）
	假如motion model已经失效，那么首先可以尝试和最近一个关键帧去做匹配(匹配关键帧中的地图点)。
	毕竟当前帧和上一个关键帧的距离还不是很远。
	作者利用了bag of words（BoW）来加速匹配。
	
	关键帧和当前帧 均用 字典单词线性表示向量
        对应单词的 描述子 肯定比较相近 取对应单词的描述子进行匹配可以加速匹配
        
	首先，计算当前帧的BoW，并设定初始位姿为上一帧的位姿；
	其次，根据位姿和BoW词典来寻找特征匹配（参见ORB－SLAM（六）回环检测）；
	最后，利用匹配的特征优化位姿（参见ORB－SLAM（五）优化）。也就是计算变换矩阵

【3】重定位（Relocalization） 跟踪
    当前帧 用词典计算 字典单词线性表示向量
    所有关键帧 用词典计算 字典单词线性表示向量

    计算 当前帧 的字典单词线性表示向量 和 所有关键帧 的 字典单词线性表示向量之间的距离 选取部分距离短的候选关键帧
    当前帧和 候选关键帧 分别进行描述子 匹配

    关键帧和 当前帧 均用 字典单词线性表示向量
    对应单词的 描述子 肯定比较相近 取对应单词的描述子进行匹配可以加速匹配

    假如当前帧与最近邻关键帧的匹配也失败了，意味着此时当前帧已经丢了，无法确定其真实位置。
    此时，只有去和所有关键帧匹配，看能否找到合适的位置。首先，计算当前帧的Bow向量。
    其次，利用BoW词典选取若干关键帧作为备选（参见ORB－SLAM（六）回环检测）；
    再次，寻找有足够多的特征点匹配的关键帧；最后，利用特征点匹配迭代求解位姿（RANSAC框架下，
    因为相对位姿可能比较大，局外点会比较多）。
    如果有关键帧有足够多的内点，那么选取该关键帧优化出的位姿。

　1）优先选择通过恒速运动模型，从LastFrame（上一普通帧）
　      直接预测出（乘以一个固定的位姿变换矩阵）当前帧的姿态；
    2）如果是静止状态或者运动模型匹配失效
	  （运用恒速模型后反投影发现LastFrame的地图点和CurrentFrame的特征点匹配很少），
	  通过增大参考帧的地图点反投影匹配范围，获取较多匹配后，计算当前位姿；
    3）若这两者均失败，即代表tracking失败，mState!=OK，
	  则在KeyFrameDataBase中用Bow搜索CurrentFrame的特征点匹配，
	  进行全局重定位GlobalRelocalization，在RANSAC框架下使用EPnP求解当前位姿。  
	  
    一旦我们通过上面三种模型获取了初始的相机位姿和初始的特征匹配，
    就可以将完整的地图投影到当前帧中去搜索更多的匹配。但是投影完整的地图，
    在large scale的场景中是很耗计算而且也没有必要的，
    因此，这里使用了局部地图LocalMap来进行投影匹配。
      
LocalMap包含：
    与当前帧相连的关键帧K1，以及与K1相连的关键帧K2（一级二级相连关键帧）；
    K1、K2对应的地图点；参考关键帧Kf。
    
匹配过程如下：
    对局部地图点
　　1. 抛弃投影范围超出相机画面的；
　　2. 抛弃观测视角和地图点平均观测方向相差60o以上的；
　　3. 抛弃特征点的尺度和地图点的尺度（通过高斯金字塔层数表示）不匹配的；
　　4. 计算当前帧中特征点的尺度；
　　5. 将地图点的描述子和当前帧ORB特征的描述子匹配，需要根据地图点尺度在初始位姿获取的粗略x投影位置附近搜索；
　　6. 根据所有匹配点进行PoseOptimization优化。 
　　
这三种跟踪模型都是为了获取相机位姿一个粗略的初值，
后面会通过跟踪局部地图TrackLocalMap对位姿进行BundleAdjustment（捆集调整），
进一步优化位姿。

************************************************************/
void Tracking::Track()
{
    // mState 为 tracking的状态机
    // SYSTME_NOT_READY , NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
    // 如果图像复位过、或者第一次运行，则为 NO_IMAGE_YET 状态
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    // 对地图上锁， 不允许更新
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    /***** 步骤1：前一帧的跟踪, 系统未初始化 进行初始化 得到初始化位姿(跟踪估计运动) *****/
    if(mState==NOT_INITIALIZED)
    {
        // 1. 得到第一帧看到的关键点
        if(mSensor==System::STEREO || mSensor==System::RGBD)
		    // 当前帧 特征点个数 大于500 进行初始化
		    // 设置第一帧为关键帧  位姿为 [I 0] 
		    // 根据第一帧视差求得的深度 计算3D点
		    // 生成地图 添加地图点 地图点观测帧 地图点最好的描述子 更新地图点的方向和距离 
		    // 关键帧的地图点 当前帧添加地图点  地图添加地图点
		    // 显示地图  
            StereoInitialization(); // 初始化双目/RGBD建图
        else
            MonocularInitialization();

        // 2. 可视化显示当前图像帧的位姿
        mpFrameDrawer->Update(this); // 更新FrameDrawer里面的数据

        if(mState!=OK)
            return;
    }

    // 1. 系统已经初始化(地图中已经有3d点) 跟踪上一帧 特征点对 计算相机位姿
    // 2. 跟踪局部地图, 图优化对位姿进行精细化调整
    // 3. 跟踪失败后的处理（两两跟踪失败 or 局部地图跟踪失败）
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // 在viewer中有个开关 menuLocalizationMode ，有它控制是否 ActivateLocalizationMode ，并最终管控mbOnlyTracking
        // A: mbOnlyTracking等于false表示正常VO模式（有地图更新），B: mbOnlyTracking等于true表示用户手动选择定位模式(已有地图，只进行跟踪和重定位)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // 检查并更新上一帧被替换的MapPoints
                // 更新Fuse函数和 SearchAndFuse 函数替换的 MapPoints	      
                CheckReplacedInLastFrame();// 上一帧地图点 是否有替换点 有替换点的则进行更新

                // a. 跟踪参考帧模式 移动速度小========================================
                    // 没有移动 跟踪参考关键帧(运动模型是空的)  或 刚完成重定位
                    // mCurrentFrame.mnId < mnLastRelocFrameId+2这个判断不应该有
                    // 应该只要mVelocity不为空，就优先选择TrackWithMotionModel
                    // mnLastRelocFrameId 上一次重定位的那一帧
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    // 将上一帧的位姿作为当前帧的初始位姿
                    // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame(); // 跟踪参考关键帧 中的 地图点个数 大于10个 返回真
                }

                // b. 有移动，先使用匀速运动模型
                else
                {
                    // 根据匀速模型 推算 当前帧的初始位姿
                    // 通过投影的方式在 上一帧参考帧 中找 当前帧特征点 的 对应匹配点
                    // 优化每个 特征点所 对应3D点的投影误差 即可得到位姿（位姿刚体变换， 有累计误差）
                    bOK = TrackWithMotionModel(); // 移动跟踪模式, 跟踪上一帧
                    if(!bOK) // 运动跟踪失败， 跟踪参考帧
                        // 不能根据匀速运动模型预测当前帧的位姿态，通过bow加速匹配（SearchByBow）
                        // 最后通过优化得到优化后的位姿		
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                // 位姿跟踪失败， 重定位
                bOK = Relocalization();
            }
        }
        // B: 已经有地图的情况下，则进行 跟踪 + 重定位（跟踪丢失后进行重定位）
        else
        {
            // Only Tracking: Local Mapping is deactivated
            // 重定位 BOW搜索 -> PnP 3d-2d匹配 求解位姿
            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                // mbVO 为0表示此帧匹配了很多的MapPoints，跟踪很正常
                // mbVO 为1表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏      
                if(!mbVO)
                {
                    // a. 上一帧跟踪的点足够多=============================================
                    // 1. 移动跟踪模式, 如果失败，尝试使用跟踪参考帧模式====
                    if(!mVelocity.empty())// 在移动
                    {
                        bOK = TrackWithMotionModel();// 恒速跟踪上一帧 模型
                        if(!bOK)// 新添加，如果移动跟踪模式失败，尝试使用 跟踪参考帧模式 进行跟踪
                            bOK = TrackReferenceKeyFrame();
                    }
                    // 2. 使用跟踪参考帧模式===============================
                    else // 未移动
                    {
                        bOK = TrackReferenceKeyFrame();// 跟踪 参考帧
                    }
                }
                // b. 上一帧跟踪的点比较少(到了无纹理区域等)，既做跟踪又做定位
                else
                {
                    // 使用 运动跟踪 和 重定位模式 计算两个位姿，如果重定位成功，使用重定位得到的位姿
                    // 即跟踪又定位， 在已有地图
                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM; // 视觉里程计跟踪得到的位姿
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();// 运动跟踪模式跟踪上一帧 结果
                        vpMPsMM = mCurrentFrame.mvpMapPoints;// 地图点
                        vbOutMM = mCurrentFrame.mvbOutlier;  // 外点
                        TcwMM = mCurrentFrame.mTcw.clone();  // 保存视觉里程计 位姿 结果 世界坐标系中相机的位姿
                    }
                    bOKReloc = Relocalization();

                    // 重定位没有成功， 但运动跟踪成功，
                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM); // 将运动模型的结果作为当前帧的位姿
                        mCurrentFrame.mvpMapPoints = vpMPsMM; // 地图点
                        mCurrentFrame.mvbOutlier = vbOutMM; // 外点

                        if(mbVO)
                        {
                            // 更新当前帧的MapPoints被观测程度
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    // 重定位成功
                    else if(bOKReloc)
                    {
                        mbVO = false; // 只要重定位成功整个跟踪过程正常进行（清除标志位）
                    }

                    bOK = bOKReloc || bOKMM; // 重定位标志 / 运动跟踪标志
                }
            }
        }


    /***** 步骤2. 局部地图跟踪（优化) *****/	
	      // 通过之前的计算，已经得到一个对位姿的初始估计，我们就能透过投影，
	      // 从已经生成的地图点 中找到更多的对应关系，来精确结果
	      // 三种模式的初始跟踪之后  进行  局部地图的跟踪
	      // 局部地图点的描述子 和 当前帧 特征点(还没有匹配到地图点的关键点) 进行描述子匹配
	      // 图优化进行优化  利用当前帧的特征点的像素坐标和 与其匹配的3D地图点  在其原位姿上进行优化
	      // 匹配优化后 成功的点对数 一般情况下 大于30 认为成功
	      // 在刚进行过重定位的情况下 需要大于50 认为成功
        mCurrentFrame.mpReferenceKF = mpReferenceKF; // 关键参考帧

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // 步骤2.1：在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿（局部优化）
        // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
        // 在上面两两帧跟踪（恒速模型跟踪上一帧、跟踪参考帧），
        // 这里搜索局部关键帧 后 搜集所有局部MapPoints，
        // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        // 有建图线程
        if(!mbOnlyTracking) // 跟踪 + 建图 + 重定位
        {
            if(bOK)
                bOK = TrackLocalMap(); // 局部地图跟踪 g2o优化
        }
        // 无建图线程
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO) // 跟踪 + 重定位 （重定位成功）
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this); // 更新FrameDrawer里面的数据

        // If tracking were good, check if we insert a keyframe
        // 步骤2.2 局部地图跟踪成功, 根据运动模型，清除外点等，检查是否需要创建新的关键帧
        if(bOK)
        {
            // Update motion model
            // a. 有运动，则更新运动模型 Update motion model 运动速度 为前后两针的 变换矩阵
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // 显示当前相机的位姿
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean temporal point matches
            // b. 清除 UpdateLastFrame 中为当前帧 临时添加的 MapPoints	    
            // 当前帧 的 地图点的 观测帧数量小于1 剔除 相应的 地图点
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; //当前帧 匹配到的地图点
                if(pMP)
                    if(pMP->Observations()<1) // 其观测帧 小于 1
                    {
                        mCurrentFrame.mvbOutlier[i]=false;// 外点标志 0 
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);// 清掉 相应的 地图点
                    }
            }

            // Delete temporal MapPoints
            // c. 清除临时的MapPoints,删除临时的地图点
            // 这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）
            // b步骤 中只是在 当前帧 中将这些 MapPoints 剔除，这里从 MapPoints数据库 中删除
            // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            //  list<MapPoint*>::iterator 
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP; // 剔除地图点对应的空间
            }
            mlpTemporalPoints.clear(); // 临时点集合 记录 链表清空

            // Check if we need to insert a new keyframe
            // d. 判断是否需要新建关键帧
            // 最后一步是确定是否将当前帧定为关键帧，由于在Local Mapping中，
            // 会剔除冗余关键帧，所以我们要尽快插入新的关键帧，这样才能更鲁棒。
            if(NeedNewKeyFrame())
            {
                CreateNewKeyFrame();
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // e. 外点清除 检查外点 标记(不符合 变换矩阵的 点 优化时更新)   
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // 3. 跟踪失败后的处理（两两跟踪失败 or 局部地图跟踪失败）
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5) // 关键帧数量过少（刚开始建图） 直接退
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame); // 新建关键帧
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // 步骤3: 返回跟踪得到的位姿 信息
    // 计算参考帧到当前帧 的变换 Tcr = mTcw  * mTwr 
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else //跟踪丢失 会造成  位姿为空（存入上一次的位姿）
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}
/*
完整跟踪：
         1. 系统开始前两帧用来初始化（单目/双目/RGBD）
         2. 后面两帧之间的跟踪
                     a. 建图+定位模式
                           检查并更新上一帧
                           正常：跟踪参考帧 / 跟踪上一帧(运动模式)
                           丢失：重定位
                     b. 仅定位模式
                           丢失：重定位
                           正常：
                                跟踪的点较多： 跟踪参考帧 / 跟踪上一帧(运动模式)
                                跟踪的点少  ： 运动模式/重定位模式
         3. 局部地图跟踪( 小回环优化)
                     局部地图根系，更新速度模型，清除当前帧中不好的点，检查创建关键帧
         4. ----> 局部建图----->回环检测

*/


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

/**
 * @brief 检查上一帧中的MapPoints是否被替换
 * 核对 替换 关键帧 地图点
 * 上一帧地图点 是否有替换点 有替换点的则进行更新
 * Local Mapping线程可能会将关键帧中某些MapPoints进行替换，由于tracking中需要用到mLastFrame，这里检查并更新上一帧中被替换的MapPoints
 * @see LocalMapping::SearchInNeighbors()
 */
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0;
    int nTotal= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;
                if(mCurrentFrame.mvpMapPoints[i])
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        nMap++;
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap=1;
        nTotal=1;
    }

    const float ratioMap = (float)nMap/fmax(1.0f,nTotal);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || ratioMap<0.3f) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| ratioMap<thMapRatio) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF;
    if(mSensor == System::RGBD) // rgbd	    
        pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB,
                            this->mImRGB, this->mImDepth);
    else
        pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);
    
    mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth );

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while(!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
