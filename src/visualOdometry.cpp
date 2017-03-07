/*************************************************************************
	> File Name: rgbd-slam-tutorial-gx/part V/src/visualOdometry.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年08月01日 星期六 15时35分42秒
 ************************************************************************/


#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

#include "slamBase.h"

// 给定index，读取一帧数据
FRAME readFrame( int index, ParameterReader& pd );
// 度量运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main( int argc, char** argv )
{
    ParameterReader pd;
    //atoi 将字符串转化为整形 c_str()返回字符串的首地址
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME lastFrame = readFrame( currIndex, pd ); // 上一帧数据
    // 我们总是在比较currFrame和lastFrame
    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    
    //computeKeyPointsAndDesp 同时提取关键点与特征描述子
    computeKeyPointsAndDesp( lastFrame, detector, descriptor );
    // image2PonitCloud 将rgb图转换为点云
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
    
    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");
  
    //最小样本点
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );

    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        computeKeyPointsAndDesp( currFrame, detector, descriptor );
        // 比较currFrame 和 lastFrame
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
	
	cout<<"result.inliers = "<<result.inliers<<endl;
	
        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
            continue;
        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        cloud = joinPointCloud( cloud, currFrame, T, camera );
        
        if ( visualize == true )
            viewer.showCloud( cloud );

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile( "/home/xiaoguo/Slam/part5_orb/data/result.pcd", *cloud );
    return 0;
}

FRAME readFrame( int index, ParameterReader& pd )
{
    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");
    
    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;  
    
    string filename;
    ss>>filename;
    
    //cout<<filename<<endl;
    f.rgb = cv::imread( filename );
    
    //f.rgb = cv::imread( "/home/xiaoguo/Slam/part5_orb/data/rgb1.png" );
    //检测是否找到帧图
    //cv::imshow(filename, f.rgb );   
    
    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;
    
    //cout<<filename<<endl;
    f.depth = cv::imread( filename, -1 );
    //f.depth = cv::imread( "/home/xiaoguo/Slam/part5_orb/data/1.png", -1 );
   // cv::imshow(filename, f.depth );  
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}
