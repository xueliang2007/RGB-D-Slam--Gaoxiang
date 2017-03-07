/*************************************************************************
	> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年07月18日 星期六 15时14分22秒
    > 说明：rgbd-slam教程所用到的基本函数（C风格）
 ************************************************************************/
# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
#include <map>
#include <string>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// PCL
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/common/transforms.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>

// 类型重定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    int frameID; 
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// 参数读取类
class ParameterReader
{
public:
   ParameterReader( string filename="/home/xiaoguo/Slam/part5_orb/parameters.txt" )  
   //ParameterReader( string filename="/home/xiaoguo/Slam/part5_orb/parameters_myself.txt" )  
   //ParameterReader( string filename="/home/xiaoguo/Slam/part5_orb/parameters_SIFT.txt" ) 
    {
          /*********************************
	 * 参数文件手动修改时特征算子还需要手动选择
	 * 所以添加一个if选择
	 * *******************************/
	  
	string::size_type SIFT_OR_NOT = filename.find("SIFT");
	if( SIFT_OR_NOT != string::npos )
	{
	  cv::initModule_nonfree();
	}
	

      //ifstream文件读操作，ofstream 文件写操作，fstream读写操作
      //c_str() 返回当前字符串的首字符地址
      ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        
	
        //当特征算子是SIFT时需要调用这个函数
        //cv::initModule_nonfree();
        //eof()从输入流读取数据，如果到达文件末尾，则eof值为非零，否则为零
	while(!fin.eof())	        //即如果文件没有结束
        {
            string str;
	    //getline函数生成一个包含一串从输入流读入的字符的字符串，
	    //直到以下情况发生会导致生成的此字符串结束。1）到文件结束，2）遇到函数的定界符，3）输入达到最大限度。
            getline( fin, str );
            if (str[0] == '#') //判断首字符是不是#
            {
                // 以‘＃’开头的是注释
                continue;
            }
            //find函数 找到要找的值并返回它的位置
            int pos = str.find("="); //POS是等号的位置
            if (pos == -1) //空行时POS为-1
                continue;
	    //读取从0到POS那个位置的整个字符串
	    //例如 STORE'abcdefghijklm' To mystring
	    //SUBSTR(mystring ,1,5) 显示 "abcde"
	    //SUBSTR(mystring ,6) 显示 "fghijklm"
	    //SUBSTR(mystring,-2)显示“lm”
            string key = str.substr( 0, pos );
	    //value 是等号之后的数值
            string value = str.substr( pos+1, str.length() );
	    //data是一个map，类似键和值的对应
	    //map是一个关联容器
            data[key] = value; 
	    //fstream类的good()函数用来判断当前流状态是否健康，当遇到EOF、输入类型不匹配的时候放回false。
	    //对应的，clear()函数用于清除流状态标志。
            if ( !fin.good() )	//如果数据流出错则break
                break; 
        }
    }
    
    // 用法 int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    /************************
     * 输入 string key 
     * 输出 key在map类data中对应的键值
     * ***********************/
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
	//end()函数用find函数来定位数据出现位置，它返回的一个迭代器，当数据出现时，
	//它返回数据所在位置的迭代器，如果map中没有要查找的数据，它返回的迭代器等于end函数返回的迭代器
        if (iter == data.end()) //即如果没有找到那个字符串则进入if输出
        {
	  //Cerr用于输出错误信息
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        //second 详见 https://zhidao.baidu.com/question/455401072.html
        return iter->second;
    }
public:
    map<string, string> data;
};

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera; 
    //atof将字符串转化为float
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}


//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */