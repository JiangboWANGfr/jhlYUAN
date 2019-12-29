#ifndef ARMORDETECTION_H
#define ARMORDETECTION_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <math.h>
#include <iostream>


typedef struct ARMOR
{
    cv::Point2f center;
    cv::Point2f rect[4];
}Armor;
typedef struct ANGLE
{
    float yaw;
    float pitch;
    float x;
    float y;
    float z;
    bool find_armor;
}Angle;
// typedef struct ANGLEKF
// {
//     float yaw;
//     float pitch;
//     float x;
//     float y;
//     float z;
    
// }AngleKF;

using namespace std;
using namespace cv;

class armordetection
{
public:

    void setup();
    void cut(cv::Mat &g_srcImage);
    void Bright();
    void Bright(int alpha , int nThresh);
    void BrighttoCanny();
    void BrighttoCanny(int CannyLowThreshold);
    void ShowAreaRect();
    void filter();
    void find_targetpoints(cv::RotatedRect &armorlamppost_left, cv::RotatedRect &armorlamppost_right);
    double calcLineDegree(const cv::Point2f& firstPt, const cv::Point2f& secondPt);
    double getRcDegree(const cv::RotatedRect box);
    static inline bool xCmp(float &x1, float &x2);
    
    cv::Mat srcImage ;
//    cv::Mat showImage;
    std::vector<cv::Mat> rgbChannels;
    bool R_B;
    std::vector<std::vector<cv::Point> > BrighttoCannyContours;
    std::vector<cv::Vec4i> BrighttoCannyHierarchy;
    cv::Mat thresholdImageRed;
    cv::Mat thresholdImageBlue;
//    cv::Mat BrighttoCannyImage;
//    cv::Mat filterRectImage;
    std::vector<cv::RotatedRect> armorDetectionLeft;
    std::vector<cv::RotatedRect> armorDetectionRight;
    std::vector<bool> armorSize;
    std::vector<int> filteredcenter;
    std::vector<bool> armor_frame;
    std::vector<cv::Point2i> chasecenter;
    std::vector<int>     chasedistance;
    int lens = 9;
    float disMatrix[9] = {0};
    Armor target;
    Angle pnpresult;
    // AngleKF kfresult;
    bool armor_location;
    bool B_S;
    float armor_size;
    cv::Point2i pre_pt;
    int width;
    int height;

    // int stateNum;                                      //状态值6×1向量(x,y,z,v_x,v_y,v_z)
    // int measureNum;                                    //测量值3×1向量(x,y,z)
    // cv::KalmanFilter KF;
    // cv::Mat measurement;

private:
    cv::Mat Kern;
};

void armordetection::setup()
{
    srcImage = cv::Mat::zeros(1280, 1024, CV_8UC3);
//    showImage = cv::Mat::zeros(height,width, CV_8UC3);
    thresholdImageRed = cv::Mat::zeros(1280, 1024, CV_8UC3);
    thresholdImageBlue = cv::Mat::zeros(1280, 1024, CV_8UC3);
//    BrighttoCannyImage = cv::Mat::zeros(height, width, CV_8UC3);
//    filterRectImage = cv::Mat::zeros(height, width, CV_8UC3);

    BrighttoCannyContours.resize(0);
    BrighttoCannyHierarchy.resize(0);
    chasecenter.resize(0);
    chasedistance.resize(0);
    armor_frame.resize(0);
    filteredcenter.resize(0);
    armorDetectionLeft.resize(0);
    armorDetectionRight.resize(0);
    rgbChannels.resize(3);
    R_B = false;
    B_S = true;
    armor_location = false;
    armor_size = 1.0;
    pre_pt.x = 0;
    pre_pt.y = 0;
    pnpresult.pitch = 0;
    pnpresult.yaw = 0;
    pnpresult.x = 0;
    pnpresult.y = 0;
    pnpresult.z = 0;
    pnpresult.find_armor = false;
    // kfresult.pitch = 0;
    // kfresult.yaw = 0;
    // kfresult.x = 0;
    // kfresult.y = 0;
    // kfresult.z = 0;
    target.center.x = 0;
    target.center.y = 0;
    armor_size = 1.0;
    width = 640;
    height = 480;

    // stateNum = 6;                                      //状态值6×1向量(x,y,z,v_x,v_y,v_z)
    // measureNum = 3;                                    //测量值3×1向量(x,y,z)
    // KF.init(stateNum, measureNum, 0);
    // KF.transitionMatrix = (Mat_<float>(6, 6) << 1,0,0,0.016,0,0,0,
	// 					                        0,1,0,0,0.016,0,
	// 					                        0,0,1,0,0,0.016,
	// 					                        0,0,0,1,0,0,
	// 					                        0,0,0,0,1,0,
	// 					                        0,0,0,0,0,1);  //转移矩阵A
    // cv::setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    // cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3));                            //系统噪声方差矩阵Q
    // cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1));                        //测量噪声方差矩阵R
    // cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));                                  //后验错误估计协方差矩阵P
    // KF.statePost = (Mat_<float>(6, 1) << 0,0,0,0,0,0);   //初始状态值x(0)
    // measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义

    Kern = (Mat_<char>(3,3) <<1,1,1,
                              1,1,1,
                              1,1,1 );//卷积核 具有增强效果
        /** @name效果不错的卷积核
          1,1,1,
          1,1,1,
          1,1,1

          1,0,1,
          0,1,0,
          1,0,1
        */
    
}

void armordetection::cut(cv::Mat &g_srcImage)
{
//    imshow("src",g_srcImage);
    cv::Mat cutImage;
//    g_srcImage.copyTo(cutImage);

    cv::Point2i center_point;
    cv::Point2i cur_pt;

    if(armor_location)
    {
        center_point.x = chasecenter.back().x;
        center_point.y = chasecenter.back().y;
        width = 640;
        height = 480;
//        if(width < 100 || height < 100 || width > 640 || height > 480)
//        {
//            width = 640;
//            height = 480;
//        }
//        cout << "height = " << height << endl;
//        cout << "width = " << width << endl ;
    }
    else
    {
        armor_size = 1.0;
        center_point.x = 640;
        center_point.y = 512;
        width = 1280;
        height = 1024;
    }

    srcImage.release();
//    showImage.release();
    thresholdImageRed.release();
    thresholdImageBlue.release();
//    BrighttoCannyImage.release();
//    filterRectImage.release();



    cur_pt.x = center_point.x + width / 2;
    cur_pt.y = center_point.y + height / 2;
    pre_pt.x = center_point.x - width / 2;
    pre_pt.y = center_point.y - height / 2;

    //cout << "center_point:" << center_point << endl;

    if(pre_pt.x < 0)
    {
        pre_pt.x = 0;
    }
    if(pre_pt.y < 0)
    {
        pre_pt.y = 0;
    }
    if(cur_pt.x > 1280)
    {
        pre_pt.x = pre_pt.x - (cur_pt.x - 1280);
    }
    if(cur_pt.y > 1024)
    {
        pre_pt.y = pre_pt.y - (cur_pt.y - 1024);
    }


    srcImage = g_srcImage(Rect(pre_pt.x,pre_pt.y,width,height));
}
//改变亮度并卷积加强边缘
void armordetection::Bright()

{
    int alpha = 10;
    int nThresh = 255;
    //改变图像亮度
    cv::Mat Bright_image = srcImage;
    double Bright_alpha = alpha/10;

    cv::Mat Bright_change;
    Bright_image.convertTo(Bright_change,-1,Bright_alpha,nThresh-255);
	
    //分离通道
    cv::split(Bright_change,rgbChannels);

    //卷积处理 卷积核为Kern

    //cv::filter2D(rgbChannels[0], rgbChannels[0], rgbChannels[0].depth(), Kern);

    //cv::filter2D(rgbChannels[2], rgbChannels[2], rgbChannels[2].depth(), Kern);
    //先腐蚀后膨胀处理
    //cv::Mat element1, element2;

    //element1 = cv::getStructuringElement(MORPH_RECT, Size(1, 5));
    //element2 = cv::getStructuringElement(MORPH_RECT, Size(3, 7));

    //cv::erode(rgbChannels[0],rgbChannels[0],element1,Point(-1,-1),1);
    //cv::erode(rgbChannels[2],rgbChannels[2],element1,Point(-1,-1),1);

//    imshow("RED_e",rgbChannels[2]);
//    imshow("BLUE_e",rgbChannels[0]);

    //cv::dilate(rgbChannels[0],rgbChannels[0],element2,Point(-1,-1),1);
    //cv::dilate(rgbChannels[2],rgbChannels[2],element2,Point(-1,-1),1);

//    imshow("RED",rgbChannels[2]);
//   imshow("BLUE",rgbChannels[0]);

}

void armordetection::Bright(int alpha, int nThresh)

{
    //改变图像亮度
    cv::Mat Bright_image = srcImage;
    double Bright_alpha = alpha/10;

    cv::Mat Bright_change;
    Bright_image.convertTo(Bright_change,-1,Bright_alpha,nThresh-255);
    
    //分离通道
    cv::split(Bright_change,rgbChannels);

    //卷积处理 卷积核为Kern

    cv::filter2D(rgbChannels[0], rgbChannels[0], rgbChannels[0].depth(), Kern);

    cv::filter2D(rgbChannels[2], rgbChannels[2], rgbChannels[2].depth(), Kern);
    //先腐蚀后膨胀处理
    //cv::Mat element1, element2;

    //element1 = getStructuringElement(MORPH_RECT, Size(1, 5));
    //element2 = getStructuringElement(MORPH_RECT, Size(3, 7));

    //cv::erode(rgbChannels[0],rgbChannels[0],element1,Point(-1,-1),1);
    //cv::erode(rgbChannels[2],rgbChannels[2],element1,Point(-1,-1),1);

//    cv::imshow("RED_e",rgbChannels[2]);
//    cv::imshow("BLUE_e",rgbChannels[0]);

    //cv::dilate(rgbChannels[0],rgbChannels[0],element2,Point(-1,-1),1);
    //cv::dilate(rgbChannels[2],rgbChannels[2],element2,Point(-1,-1),1);

//    cv::imshow("RED",rgbChannels[2]);
//    cv::imshow("BLUE",rgbChannels[0]);

}


void armordetection::BrighttoCanny()
//卷积后Canny处理并输出轮廓
{
    int CannyLowThreshold = 150;

    cv::threshold(rgbChannels[0],thresholdImageBlue,80,255,THRESH_BINARY);
    cv::threshold(rgbChannels[2],thresholdImageRed,80,255,THRESH_BINARY);

    //Canny边缘检测&寻找轮廓
    if(R_B)
    {
        cv::Canny(thresholdImageBlue,thresholdImageBlue, CannyLowThreshold, CannyLowThreshold*3, 3);
        cv::findContours(thresholdImageBlue,BrighttoCannyContours,BrighttoCannyHierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
    }
    else
    {
        cv::Canny(thresholdImageRed,thresholdImageRed, CannyLowThreshold, CannyLowThreshold*3, 3);
        cv::findContours(thresholdImageRed,BrighttoCannyContours,BrighttoCannyHierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
    }

}

void armordetection::BrighttoCanny(int CannyLowThreshold)
//卷积后Canny处理并输出轮廓
{
    cv::threshold(rgbChannels[0],thresholdImageBlue,160,255,THRESH_BINARY);
    cv::threshold(rgbChannels[2],thresholdImageRed,160,255,THRESH_BINARY);

    //Canny边缘检测&寻找轮廓
    if(R_B)
    {
        cv::Canny(thresholdImageBlue,thresholdImageBlue, CannyLowThreshold, CannyLowThreshold*3, 3);
        cv::findContours(thresholdImageBlue,BrighttoCannyContours,BrighttoCannyHierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
    }
    else
    {
        cv::Canny(thresholdImageRed,thresholdImageRed, CannyLowThreshold, CannyLowThreshold*3, 3);
        cv::findContours(thresholdImageRed,BrighttoCannyContours,BrighttoCannyHierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
    }
}

//旋转矩形包围轮廓方便观察
void armordetection::ShowAreaRect()
{
    cv::RotatedRect minRect;
    cv::Mat minAreaRectImage = cv::Mat::zeros(srcImage.size(), CV_8UC3);
    for(int j = 0 ; j < BrighttoCannyContours.size(); j++)
    {
        //最小旋转矩形包围轮廓并输出四顶点
        minRect = cv::minAreaRect(cv::Mat(BrighttoCannyContours[j]));
        cv::Point2f rect_points[4];// The order is bottomRight, bottomLeft, topLeft, topRight
        minRect.points(rect_points);


        //画旋转矩形
        for (int k = 0; k < 4; k++)
        {
            cv::line(minAreaRectImage, rect_points[k], rect_points[(k + 1) % 4],cv::Scalar(255,0,255), 2);
        }
    }

    cv::namedWindow("minAreaRect",WINDOW_AUTOSIZE);
    cv::imshow("minAreaRect",minAreaRectImage);
}

double armordetection::calcLineDegree(const cv::Point2f& firstPt, const cv::Point2f& secondPt)
{
    double curLineAngle = 0.0f;
    if (secondPt.x - firstPt.x != 0)
    {
        curLineAngle = std::atan(static_cast<double>(firstPt.y - secondPt.y) / static_cast<double>(secondPt.x - firstPt.x));
        if (curLineAngle < 0)
        {
            curLineAngle += CV_PI;
        }
    }
    else
    {
        curLineAngle = CV_PI / 2.0f; //90度
    }
    return curLineAngle*180.0f/CV_PI;
}
double armordetection::getRcDegree(const cv::RotatedRect box)
{
    double degree = 0.0f;
    cv::Point2f vertVect[4];
    box.points(vertVect);
    //line 1
    const double firstLineLen = (vertVect[1].x - vertVect[0].x)*(vertVect[1].x - vertVect[0].x) +
        (vertVect[1].y - vertVect[0].y)*(vertVect[1].y - vertVect[0].y);
    //line 2
    const double secondLineLen = (vertVect[2].x - vertVect[1].x)*(vertVect[2].x - vertVect[1].x) +
        (vertVect[2].y - vertVect[1].y)*(vertVect[2].y - vertVect[1].y);
    if (firstLineLen > secondLineLen)
    {
        degree = calcLineDegree(vertVect[0], vertVect[1]);
    }
    else
    {
        degree = calcLineDegree(vertVect[2], vertVect[1]);
    }
    return degree;
}


void armordetection::find_targetpoints(cv::RotatedRect &armorlamppost_left, cv::RotatedRect &armorlamppost_right)
{
    cv::Point2f frect_points[4];
    std::vector<cv::Point2f> top_points(0);
    std::vector<cv::Point2f> bottom_points(0);
    armorlamppost_left.points(frect_points);
    for(int i = 0; i < 4; i++)
    {
        if(frect_points[i].y < armorlamppost_left.center.y)
            top_points.push_back(frect_points[i]);
        else
            bottom_points.push_back(frect_points[i]);
    }

//    double degree_left = getRcDegree(armorlamppost_left);

//    float h = armorlamppost_left.size.height > armorlamppost_left.size.width? armorlamppost_left.size.height: armorlamppost_left.size.width;
//    Point2f l_l;
//    l_l.x = h * cos(degree_left)/2;
//    l_l.y = -h * sin(degree_left)/2;
    target.rect[0] = Point2i((top_points[0] + top_points[1])/2);
    //std::cout << "target.rect[0]=" << target.rect[0] << std::endl;
    target.rect[1] = Point2i((bottom_points[0] + bottom_points[1])/2);
    //std::cout << "target.rect[1]=" << target.rect[1] << std::endl;

    armorlamppost_right.points(frect_points);
    for(int i = 0; i < 4; i++)
    {
        if(frect_points[i].y < armorlamppost_right.center.y)
            top_points.push_back(frect_points[i]);
        else
            bottom_points.push_back(frect_points[i]);
    }
//    double degree_right = getRcDegree(armorlamppost_right);
//    h = armorlamppost_right.size.height > armorlamppost_right.size.width? armorlamppost_right.size.height: armorlamppost_right.size.width;
//    Point2f l_r;
//    l_r.x = h * cos(degree_right)/2;
//    l_r.y = -h * sin(degree_right)/2;
    target.rect[2] = Point2i((top_points[2] + top_points[3])/2);
    //std::cout << "target.rect[2]=" << target.rect[2] << std::endl;
    target.rect[3] = Point2i((bottom_points[2] + bottom_points[3])/2);
    //std::cout << "target.rect[3]=" << target.rect[3] << std::endl;
    char tam0[100],tam1[100],tam2[100],tam3[100];
    // std::sprintf(tam0, "0 = (%0.f,%0.f) ",target.rect[0].x, target.rect[0].y);
    // std::sprintf(tam1, "1 = (%0.f,%0.f) ",target.rect[1].x, target.rect[1].y);
    // std::sprintf(tam2, "2 = (%0.f,%0.f) ",target.rect[2].x, target.rect[2].y);
    // std::sprintf(tam3, "3 = (%0.f,%0.f) ",target.rect[3].x, target.rect[3].y);
    // cv::circle(srcImage,target.rect[0],3,cv::Scalar(0,0,255),3);
    // cv::circle(srcImage,target.rect[1],3,cv::Scalar(0,0,255),3);
    // cv::circle(srcImage,target.rect[2],3,cv::Scalar(0,0,255),3);
    // cv::circle(srcImage,target.rect[3],3,cv::Scalar(0,0,255),3);
    // cv::putText(srcImage,tam0, target.rect[0], FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
    // cv::putText(srcImage,tam1, target.rect[1], FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
    // cv::putText(srcImage,tam2, target.rect[2], FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
    // cv::putText(srcImage,tam3, target.rect[3], FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);    //zjh 2019.7.5
}


void armordetection::filter()
//旋转矩形包围轮廓并筛选出合适的矩形对
{
    std::vector<cv::RotatedRect> filteredRect (0);
    cv::Point2f frect_points[4];
    float real_angle;
//    filterRectImage = srcImage.clone();
//    showImage = srcImage.clone();
    int counter = 0;

    //遍历轮廓
    for(int m = 0 ; m < BrighttoCannyContours.size();m++)
    {
        cv::RotatedRect minRotatedRect = cv::minAreaRect(BrighttoCannyContours[m]);
        minRotatedRect.points(frect_points);
        real_angle = getRcDegree(minRotatedRect);
        float width = minRotatedRect.size.width;
        float height = minRotatedRect.size.height;

        if( width > height )
        {
            std::swap(width,height);
        }

        //调取点参数
        cv::Point2i Contour_center;
        Contour_center = frect_points[0];

        //初步筛选旋转矩阵并将符合标准的旋转矩阵另存在filteredRect
        std::vector<float> s(7);//s = size 外形标准
        s[0] = height/width;
        s[1] = height*width;
        s[2] = minRotatedRect.angle;
        s[3] = real_angle;
	
        s[4] = std::sqrt(std::pow(frect_points[1].y - frect_points[2].y,2) + std::pow(frect_points[1].x - frect_points[2].x,2));//real height
        s[5] = std::sqrt(std::pow(frect_points[1].y - frect_points[0].y,2) + std::pow(frect_points[1].x - frect_points[0].x,2));//real width

        bool armor_exist;
	//cout << s[0] << endl;
        //长度 面积 角度初步筛选
        if( (s[0] < 20 ||s[0] > 1.2) && (1) && (s[1] > 20) && ( s[3] > 70 && s[3] < 110) )
        {
            // cout << "s[3]" << s[3] << endl;
            //判断颜色
            if(R_B)
            {
//                cout << "cont" << Contour_center<< endl;
                int red = rgbChannels[2].at<uchar>(Contour_center);
               //cout << "red" <<  red << endl;
                if( red < 10 )
                    armor_exist = true;
            }
            else
            {
//                cout << "cont" << Contour_center<< endl;
                int blue = rgbChannels[0].at<uchar>(Contour_center);
                //cout << "blue" <<  blue << endl;
                if( blue < 10 )
                    armor_exist = true;
            }

            if(armor_exist)
            {
                armor_exist = false;
                for (int k = 0; k < 4; k++)
                {
                    cv::line(srcImage, frect_points[k], frect_points[(k + 1) % 4],cv::Scalar(255,0,0), 2);
                }    
                filteredRect.push_back( minRotatedRect);
                counter++;

                 //标中心点 筛选颜色后
                 //cv::circle(srcImage, minRotatedRect.center, 7, cv::Scalar(0, 0, 255), 7);
            }
        }
        else 
        {
             //cout << "s[0]: " << s[0] << endl;
             //cout << "s[1]: " << s[1] << endl;
             //cout << "s[2]: " << s[2] << endl;
             //cout << "s[3]: " << s[3] << endl;
            // cout << "s[4]: " << s[4] << endl;
            // cout << "s[5]: " << s[5] << endl;
        }
    }

    //寻找合适的矩形对
    float length,length_x,length_y;//相对位置
    float height_quotient;//高度比
    float width_quotient;//宽度比
    float a_min;
    float a_max;
    float b_min;
    float b_max;
    double a_real_angle;
    double b_real_angle;
    int theta_angle;//相对角度
    float line_angle;
    int Rects_number = 0;


    armorDetectionLeft.clear();
    armorDetectionRight.clear();
    armorSize.clear();
 
    if( counter > 1 )
    {
        //冒泡
        for(int a = 0; a < counter ;a++)
        {
            for(int b = a; b < counter ;b++)
            {
                a_min = (filteredRect[a].size.height>filteredRect[a].size.width) ? filteredRect[a].size.width : filteredRect[a].size.height;
                a_max = (filteredRect[a].size.height>filteredRect[a].size.width) ? filteredRect[a].size.height : filteredRect[a].size.width;
                b_min = (filteredRect[b].size.height>filteredRect[b].size.width) ? filteredRect[b].size.width : filteredRect[b].size.height;
                b_max = (filteredRect[b].size.height>filteredRect[b].size.width) ? filteredRect[b].size.height : filteredRect[b].size.width;
                a_real_angle = getRcDegree(filteredRect[a]);
                b_real_angle = getRcDegree(filteredRect[b]);
                length_x = filteredRect[a].center.x - filteredRect[b].center.x;
                length_y = filteredRect[a].center.y - filteredRect[b].center.y;
                length = std::sqrt(length_x * length_x + length_y * length_y);
                theta_angle = std::abs(a_real_angle - b_real_angle);
                height_quotient = b_max/a_max;
                width_quotient = b_min/a_min;

		float length_max,length_min,theta_angle_max;
        if(B_S)
        {
            length_max = 5.8;
            length_min = 3.6;
            theta_angle_max = 12;
            cout << "B" << endl;
        }
        else
        {
            length_max = 2.75;
            length_min = 1.0;
            theta_angle_max = 12;
                        cout << "S" << endl;

        }
        
    
              if( length > a_max * length_min && length > b_max* length_min && length < a_max* length_max && length < b_max* length_max
                && (theta_angle < theta_angle_max )
                 && height_quotient < 2.5 && height_quotient > 0.4 && width_quotient < 2.5 && width_quotient > 0.4)
                {
                    if(abs(int(filteredRect[a].center.x - filteredRect[b].center.x)) != 0)
                        line_angle = std::abs(180 / CV_PI * int(filteredRect[a].center.y - filteredRect[b].center.y)) / std::abs(int(filteredRect[a].center.x - filteredRect[b].center.x));
                    else
                        line_angle = 90;

                    //std::cout << "a_real_angle=" << a_real_angle << endl;
                    //std::cout << "b_real_angle=" << b_real_angle << endl;
  		    //std::cout << "line_angle=" << line_angle << endl;
		    if( (line_angle + a_real_angle < 120 && line_angle + a_real_angle > 60)
                     || (line_angle + b_real_angle < 120 && line_angle + b_real_angle > 60) )
                    {
//                        cout << "line_angle=" << line_angle << endl;
                       std::cout << "theta_angle=" << theta_angle << endl;
//                        std::cout << "a_real_angle=" << a_real_angle << endl;
//                        std::cout << "b_real_angle=" << b_real_angle << endl;
                        cv::line(srcImage,filteredRect[a].center,filteredRect[b].center,cv::Scalar(255,255,0),2);
                        cv::circle(srcImage,filteredRect[a].center/2+filteredRect[b].center/2,4,cv::Scalar(0,255,0),4);
                        Rects_number++;
                         cout << "length/a_max:" << length/a_max << endl;
                         cout << "length/b_max:" << length/b_max << endl;
                        // cout << "a_max:" << a_max << endl;
                        // cout << "b_max:" << b_max << endl;
                        
			if(length > 3 * a_max && length > 3 * b_max)
                        armorSize.push_back(true);
                        else
                        armorSize.push_back(false);

                        if(filteredRect[b].center.x > filteredRect[a].center.x)
                        {
                            armorDetectionRight.push_back(filteredRect[b]);
                            armorDetectionLeft.push_back(filteredRect[a]);
                        }
                        else
                        {
                            armorDetectionRight.push_back(filteredRect[a]);
                            armorDetectionLeft.push_back(filteredRect[b]);
                        }
                    }//线->角度
		    else cout << " line_angle+real_angle > 120 <60" << endl;
                }//匹配矩形对
		    else
		    {
			    if(length > 10)
                {
                    //cout << "width_quotient: " << width_quotient << endl;
			        // cout << "height_quotient: " << height_quotient << endl;
			         cout << "theta_angle: " << theta_angle << endl;
                     cout << "length/a_max: " << length/a_max << endl;
			         cout << "length/b_max: " << length/b_max << endl;
			        // cout << "length/a_min: " << length/a_min << endl;
			        //  cout << "length/b_min: " << length/b_min << endl;
			        // cout << "length: " << length << endl;
                    // cout << "length_x: " << length_x << endl;
                    // cout << "length_y: " << length_y << endl;
                }
		    }
     
        
            }//冒泡
        }//冒泡
    }
 else cout << " counter <= 1" <<endl;


    filteredcenter.clear();

    if(!armorDetectionLeft.empty())
    {
        armor_location = true;
        armor_frame.push_back(true);

        cv::Point2i armor_center_f = armorDetectionLeft[Rects_number - 1].center/2 + armorDetectionRight[Rects_number - 1].center/2;
        //cout << "armor is at:" << armor_center_f + pre_pt << endl;
        filteredcenter.push_back(Rects_number - 1);//最后一个

        for(int p = 0 ; p < Rects_number - 1 ; p++)
        {

            for(int q = p + 1 ; q < Rects_number; q++)
            {
                cv::Point2i armor_center_p = armorDetectionLeft[p].center/2 + armorDetectionRight[p].center/2;
                cv::Point2i armor_center_q = armorDetectionLeft[q].center/2 + armorDetectionRight[q].center/2;
                if( std::abs(armor_center_p.x - armor_center_q.x) < 5 && std::abs(armor_center_p.y - armor_center_q.y) < 5)
                {
//                    cout << p << q << endl;
//                    cout << armor_center_p << armor_center_q << endl;
                break;
                }
                if( q == Rects_number - 1)
                {
                    //cout << "armor is at:" << armor_center_p + pre_pt << endl;
                    filteredcenter.push_back(p);//第p+1个


                }
            }
        }
                //cout << "the number of detected centers is:" << filteredcenter.size() << endl;

        //跟踪
        //选一个最合适的中心点
        int hh = 0;
	int score_h = 0;
	int score_hh = 0;
        for(int h = 1 ; h < filteredcenter.size() ; h++)
        {
            //相对角度
	    float delta_angle_h = getRcDegree(armorDetectionRight[filteredcenter[h]]) - getRcDegree(armorDetectionLeft[filteredcenter[h]]);
            float delta_angle_hh = getRcDegree(armorDetectionRight[filteredcenter[hh]]) - getRcDegree(armorDetectionLeft[filteredcenter[hh]]);
	    //左右比例
            float right_height_h = (armorDetectionRight[filteredcenter[h]].size.height>armorDetectionRight[filteredcenter[h]].size.width) ? armorDetectionRight[filteredcenter[h]].size.height : armorDetectionRight[filteredcenter[h]].size.width;
	    float left_height_h = (armorDetectionLeft[filteredcenter[h]].size.height>armorDetectionLeft[filteredcenter[h]].size.width) ? armorDetectionLeft[filteredcenter[h]].size.height : armorDetectionLeft[filteredcenter[h]].size.width;
	    float height_quotient_h = right_height_h / left_height_h;
	    if(height_quotient_h < 1.0)
            height_quotient_h = 1.0 / height_quotient_h;

            float right_height_hh = (armorDetectionRight[filteredcenter[hh]].size.height>armorDetectionRight[filteredcenter[hh]].size.width) ? armorDetectionRight[filteredcenter[hh]].size.height : armorDetectionRight[filteredcenter[hh]].size.width;
	    float left_height_hh = (armorDetectionLeft[filteredcenter[hh]].size.height>armorDetectionLeft[filteredcenter[hh]].size.width) ? armorDetectionLeft[filteredcenter[hh]].size.height : armorDetectionLeft[filteredcenter[hh]].size.width;
	    float height_quotient_hh = right_height_hh / left_height_hh;
	    if(height_quotient_hh < 1.0)
            height_quotient_hh = 1.0 / height_quotient_hh;
	    //面积
        float area_h = armorDetectionRight[filteredcenter[h]].size.width*armorDetectionRight[filteredcenter[h]].size.height + \
                        armorDetectionLeft[filteredcenter[h]].size.width*armorDetectionLeft[filteredcenter[h]].size.height;
        float area_hh = armorDetectionRight[filteredcenter[hh]].size.width*armorDetectionRight[filteredcenter[hh]].size.height + \
                        armorDetectionLeft[filteredcenter[hh]].size.width*armorDetectionLeft[filteredcenter[hh]].size.height;
        cout << "area_h = " << area_h << "\t" << "area_hh = " << area_hh << endl;
	    //float 
	    //cout << "delta_angle_h" << delta_angle_h << endl;
            //cout << "delta_angle_hh" << delta_angle_hh << endl;
	    //评分
	    if(abs(delta_angle_h) < abs(delta_angle_hh)) score_h++;
	    else score_hh++;
	    if(height_quotient_h < height_quotient_hh) score_h++;
	    else score_hh++;
        if(area_h > area_hh) score_h+=3;
        else score_hh+=3;
	    
            if(score_h > score_hh)
            {
                //cout << "change!!!" << endl;
                hh = h;
            }
        }
        //cout << "hh=" << hh << endl;
	cv::circle(srcImage,armorDetectionRight[filteredcenter[hh]].center/2+armorDetectionLeft[filteredcenter[hh]].center/2,8,cv::Scalar(255,0,255),8);
        //目标压入
        armordetection::find_targetpoints(armorDetectionLeft[filteredcenter[hh]], armorDetectionRight[filteredcenter[hh]]);
        //B_S = armorSize[filteredcenter[hh]];
        //设置剪切大小
        // armor_size = armorDetectionLeft[filteredcenter[hh]].size.height / 200;
        // if(armor_size < 0.1) armor_size = 0.1;
        //保存中心点
        if(chasecenter.size() < 10)//只保留后 x 个元素
        {
            chasecenter.push_back(cv::Point2i(armorDetectionLeft[filteredcenter[hh]].center/2 + armorDetectionRight[filteredcenter[hh]].center/2) + pre_pt);
        }
        else
        {            
            chasecenter.erase(chasecenter.begin());
            chasecenter.push_back(cv::Point2i(armorDetectionLeft[filteredcenter[hh]].center/2 + armorDetectionRight[filteredcenter[hh]].center/2) + pre_pt);
//            double len = sqrt(pow(armorDetectionLeft[filteredcenter[hh]].center.x - armorDetectionRight[filteredcenter[hh]].center.x,2) + pow(armorDetectionLeft[filteredcenter[hh]].center.y - armorDetectionRight[filteredcenter[hh]].center.y,2));
//            cout << len << endl;

//            double height_L;
//            double height_R;

//            if(armorDetectionLeft[filteredcenter[hh]].size.height > armorDetectionLeft[filteredcenter[hh]].size.width)
//                height_L = armorDetectionLeft[filteredcenter[hh]].size.height;
//            else
//                height_L = armorDetectionLeft[filteredcenter[hh]].size.width;
//            cout << "height_L" << height_L << endl;
//            if(armorDetectionRight[filteredcenter[hh]].size.height > armorDetectionRight[filteredcenter[hh]].size.width)
//                height_R = armorDetectionRight[filteredcenter[hh]].size.height;
//            else
//                height_R = armorDetectionRight[filteredcenter[hh]].size.width;

//            double L_distance = 9000 / height_L ;
//            double R_distance = 9000 / height_R ;
//            cout << "L_distance:" << L_distance << endl;
//            cout << "R_distance:" << R_distance << endl;

//            Point2i L_center = Point2i(armorDetectionLeft[filteredcenter[hh]].center) + pre_pt;
//            cout << "L_center:" << L_center <<endl;
//            Point2i R_center = Point2i(armorDetectionRight[filteredcenter[hh]].center) + pre_pt;
//            cout << "R_center:" << R_center <<endl;

//            Point2i cen;
//            cen.x = 540;
//            cen.y = 960;
//            double L_X = (L_center - cen).x / 1920. /1.19 ;
//            double L_Y = (L_center - cen).y / 1080. /1.19 / 1.777 ;
//            cout << "L_X:" << L_X << endl;
//            cout << "L_Y:" << L_Y << endl;
        }

        //控制点在世界坐标系中
        //按照顺时针压入，左上是第一个点
        std::vector<cv::Point3f> objP;
        cv::Mat objM;
        objP.clear();
        if(B_S)
        {
            objP.push_back(cv::Point3f(0,0,0));
            objP.push_back(cv::Point3f(229,0,0));
            objP.push_back(cv::Point3f(229,55,0));
            objP.push_back(cv::Point3f(0,55,0));
        }
       else
       {
            objP.push_back(cv::Point3f(0,0,0));
            objP.push_back(cv::Point3f(134,0,0));
            objP.push_back(cv::Point3f(134,55,0));
            objP.push_back(cv::Point3f(0,55,0));
       }
        cv::Mat(objP).convertTo(objM,CV_32F);
        //目标四个点按照顺时针压入，左上是第一个点
        std::vector<cv::Point2f> points;
        points.clear();
        points.push_back(target.rect[0] + cv::Point2f(pre_pt));
        points.push_back(target.rect[2] + cv::Point2f(pre_pt));
        points.push_back(target.rect[3] + cv::Point2f(pre_pt));
        points.push_back(target.rect[1] + cv::Point2f(pre_pt));

        //设置相机内参和畸变系统
        cv::Mat  _A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
        _A_matrix.at<double>(0, 0) = 1765.75792;            //      [ fx   0  cx ]
        _A_matrix.at<double>(1, 1) = 1765.37939;            //      [  0  fy  cy ]
        _A_matrix.at<double>(0, 2) = 645.83591;                  //      [  0   0   1 ]
        _A_matrix.at<double>(1, 2) = 502.83120;
        _A_matrix.at<double>(2, 2) = 1;
        cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1); // vector of distortion coefficients
        distCoeffs.at<double>(0,0) = -0.1054;
        distCoeffs.at<double>(0,1) = 0.16105;
        distCoeffs.at<double>(0,2) = 0.00132;
        distCoeffs.at<double>(0,3) = -0.00048;
        distCoeffs.at<double>(0,4) = 0.0;

        // cv::Mat  _A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
        // _A_matrix.at<double>(0, 0) = 2397.54474;            //      [ fx   0  cx ]
        // _A_matrix.at<double>(1, 1) = 2397.48246;            //      [  0  fy  cy ]
        // _A_matrix.at<double>(0, 2) = 1089.94897;                  //      [  0   0   1 ]
        // _A_matrix.at<double>(1, 2) = 634.81142;
        // _A_matrix.at<double>(2, 2) = 1;
        // cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1); // vector of distortion coefficients
        // distCoeffs.at<double>(0,0) = -0.45564;
        // distCoeffs.at<double>(0,1) = 0.21265;
        // distCoeffs.at<double>(0,3) = 0.00068;
        //设置旋转、平移矩阵，旋转、平移向量
        cv::Mat _R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
        cv::Mat _t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
        cv::solvePnP(objP,points,_A_matrix,distCoeffs,rvec,tvec,false,SOLVEPNP_AP3P);
        cv::Rodrigues(rvec,_R_matrix);                   // converts Rotation Vector to Matrix
        _t_matrix = tvec;                            // set translation matrix
    //    //对应照相机3D空间坐标轴    //这里会让电脑卡死
    //    vector<Point3f> reference_objP;
    //    vector<Point2f> reference_imgP;
    //    reference_objP.push_back(Point3f(65,27.5,0.0));//原点
    //    reference_objP.push_back(Point3f(90,27.5,0.0));//x轴
    //    reference_objP.push_back(Point3f(65,52.5,0.0));//y轴
    //    reference_objP.push_back(Point3f(65,27.5,25));//z轴
    //    projectPoints(reference_objP,rvec,tvec,_A_matrix,distCoeffs,reference_imgP);
    //    line(filterRectImage,reference_imgP[0],reference_imgP[1],Scalar(0,0,255),2);//红色X轴
    //    line(filterRectImage,reference_imgP[0],reference_imgP[2],Scalar(0,255,0),2);//绿色Y轴
    //    line(filterRectImage,reference_imgP[0],reference_imgP[3],Scalar(255,0,0),2);//蓝色Z轴
        //求相机在世界坐标系中的坐标
        //求旋转矩阵的转置
        double m00,m01,m02;
        double m10,m11,m12;
        double m20,m21,m22;
        m00=_R_matrix.at<double>(0,0);
        m01=_R_matrix.at<double>(0,1);
        m02=_R_matrix.at<double>(0,2);
        m10=_R_matrix.at<double>(1,0);
        m11=_R_matrix.at<double>(1,1);
        m12=_R_matrix.at<double>(1,2);
        m20=_R_matrix.at<double>(2,0);
        m21=_R_matrix.at<double>(2,0);
        m22=_R_matrix.at<double>(2,2);
        double x=0.0,y=0.0,z=0.0;
        //先减去平移矩阵
        double tempx=0.0,tempy=0.0,tempz=0.0;
        tempx=0-_t_matrix.at<double>(0,0);
        tempy=0-_t_matrix.at<double>(1,0);
        tempz=0-_t_matrix.at<double>(2,0);
        //乘以矩阵的逆
        x=m00*tempx+m10*tempy+m20*tempz;
        y=m01*tempx+m11*tempy+m21*tempz;
        z=m02*tempx+m12*tempy+m22*tempz;
        char tam1[100];
        std::sprintf(tam1, "cam in world(%0.0f,%0.0f,%0.0f)",x,y,z);
        cv::putText(srcImage, tam1, cv::Point(15, 15), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
        //求装甲板中心在相机坐标系中的坐标
        double newx=0.0,newy=0.0,newz=0.0;
       if(B_S)
       {
          newx=m00*114.5+m01*27.5+m02*0+_t_matrix.at<double>(0,0) - 76.4;   //-76.4
            newy=m10*114.5+m11*27.5+m12*0+_t_matrix.at<double>(1,0) -80;    //-80
            newz=m20*114.5+m21*27.5+m22*0+_t_matrix.at<double>(2,0);
            // std::cout << "big armor!!!" << endl;
            std::sprintf(tam1, "B");
            cv::putText(srcImage, tam1, cv::Point2i(armorDetectionLeft[filteredcenter[hh]].center/2 + armorDetectionRight[filteredcenter[hh]].center/2), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255),2);
        }
        else
        {
            newx=m00*67+m01*27.5+m02*0+_t_matrix.at<double>(0,0) - 76.4;    //76.4->35
            newy=m10*67+m11*27.5+m12*0+_t_matrix.at<double>(1,0) - 80;      //48.5->130
            newz=m20*67+m21*27.5+m22*0+_t_matrix.at<double>(2,0);
            //std::cout << "small armor!!!" << endl;
             std::sprintf(tam1, "S");
            cv::putText(srcImage, tam1, cv::Point2i(armorDetectionLeft[filteredcenter[hh]].center/2 + armorDetectionRight[filteredcenter[hh]].center/2), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255),2);
        }
       
        char tam2[100];
        std::sprintf(tam2, "center in cam(%0.0f,%0.0f,%0.0f)",newx,newy,newz);
        cv::putText(srcImage, tam2, Point(15, 30), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
        if(std::abs(newx) < 6000)
            pnpresult.x = newx;
        if(std::abs(newy) < 6000)
            pnpresult.y = newy;
        if(std::abs(newz) < 12000)
            pnpresult.z = newz;

        pnpresult.find_armor = true;

        // //修正量
        // cout << "x = " << newx << "\t" << "y = " << newy << "\t" << "z = " << newz << endl;
        // float armorDistance = sqrt(newx*newx + newy*newy + newz*newz);
        // for(int i=0; i < lens-1; i++)
        // {
        //     disMatrix[i] = disMatrix[i+1];
        // }
        // disMatrix[lens-1] = armorDistance;
        // float medianFilter[lens];
        // memset(medianFilter,0,sizeof(float));
        // memcpy(medianFilter,disMatrix,lens*sizeof(float));
        // sort(medianFilter,medianFilter+lens,xCmp);
        // // for(int j=0; j<lens; j++)
        // // {
        // //     cout << medianFilter[j] << "\t" ;
        // // }
        // float fixDis = medianFilter[(lens-1)/2];
        // cout << "distance = " << fixDis << endl;
        // newy = newy - fixDis/3500*300;
        // if(fixDis >= 4000)
        // {
        //     newy = newy - 300;
        // }
        // cout << "newy = " << newy << endl;


        //算欧拉角
        //pitch：绕x轴  roll：绕z轴 yaw：绕y轴
        float pitch,roll,yaw;
        double vec[3];
        vec[0]=newx;
        vec[1]=newy;
        vec[2]=newz;
        yaw=std::atan(vec[0]/vec[2])*180/CV_PI;
        pitch=std::atan(vec[1]/vec[2])*180/CV_PI;      //因与刘的方向相反而作修改,zjh于2019.7.2
        //cout << "pitch = " << pitch << endl;
        pnpresult.yaw = yaw;
        pnpresult.pitch = pitch;
        //  if(std::abs(yaw) > 0.04) pnpresult.yaw = yaw;
        //  else pnpresult.yaw = 0.0;
        //  if(std::abs(pitch) > 0.04) pnpresult.pitch = pitch;
        //  else pnpresult.pitch = 0.0;
        char tam3[100];
        std::sprintf(tam3, "tan yaw=%0.4f   tan pich=%0.4f",vec[0]/vec[2],vec[1]/vec[2]);
        cv::putText(srcImage, tam3, cv::Point(15, 45), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
        char tam4[100];
        std::sprintf(tam4, "yaw=%0.4f   pitch=%0.4f",pnpresult.yaw,pnpresult.pitch);
        cv::putText(srcImage, tam4, cv::Point(15, 60), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
        

    }
    else
    {
        cout << "no armor found!" << endl;
        armor_frame.push_back(false);
        pnpresult.pitch = 0.0;
        pnpresult.yaw = pnpresult.yaw * 0.95;
        pnpresult.x = 0.0;
        pnpresult.y = 0.0;
        pnpresult.z = 0.0;
        pnpresult.find_armor = false;
        if(armor_frame.size() > 3)
        for(int s = 0; s < 3 ; s++)
        {
            if(armor_frame[armor_frame.size() - 1 - s] == true)
                break;
            if(s == 2)
            {
                armor_location = false;
                pnpresult.find_armor = false;
            }
        }
        else
        {
            armor_location = false;
            pnpresult.find_armor = false;
        }
    }

    // //2.kalman prediction
    // cv::Mat prediction = KF.predict();
    // cv::Point3f predict_pt = Point3f(prediction.at<float>(0), prediction.at<float>(1), prediction.at<float>(2));   //预测值(x',y',z')
    // cv::Point3f predict_add = predict_pt + 0.3 * cv::Point3f(prediction.at<float>(3), prediction.at<float>(4), prediction.at<float>(5));  //加入速度预测
    // kfresult.x = predict_add.x;
    // kfresult.y = predict_add.y;
    // kfresult.z = predict_add.z;

    // //cout << "predict_pt" << predict_pt << endl;
    // //cout << "predict_add" << predict_add << endl;

    // float predict_yaw = std::atan(kfresult.x/kfresult.z)*180/CV_PI;
    // float predict_pitch = std::atan(kfresult.y/kfresult.z)*180/CV_PI;
    // if(std::abs(predict_yaw)>0.05) kfresult.yaw=predict_yaw;
    // else kfresult.yaw=0;
    // if(std::abs(predict_pitch)>0.05) kfresult.pitch=predict_pitch;
    // else kfresult.pitch=0;
    // char buf[100];
    // std::sprintf(buf, "v_x=%0.4f   v_y=%0.4f   v_z=%0.4f",prediction.at<float>(3),prediction.at<float>(4),prediction.at<float>(5));
    // cv::putText(srcImage, buf, cv::Point(15, 75), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,255),1);
    
    // //3.update measurement
    // measurement.at<float>(0) = (float)pnpresult.x;
    // measurement.at<float>(1) = (float)pnpresult.y;
	// measurement.at<float>(2) = (float)pnpresult.z;

    // //4.update
    // KF.correct(measurement);

    //draw

    // cv::circle(image,cv::Point(predict_pt.x,predict_pt.y),5,cv::Scalar(0,255,0),3);    //predicted point with green
    // cv::circle(image,cv::Point(),5,cv::Scalar(255,0,0),3); //current position with blue
    // cv::circle(image,predict_add,5,cv::Scalar(0,0,255),3); //predicted position with red

    // char buf[256];

    // sprintf(buf,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
    // putText(image,buf,Point(10,30),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
    // sprintf(buf,"current position :(%3d,%3d)",mouse.x,mouse.y);
    // putText(image,buf,Point(10,60),CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);

}

bool armordetection::xCmp(float &x1, float &x2)
{
    return x1 < x2;     //从小到大排序
}

#endif // ARMORDETECTION_H
