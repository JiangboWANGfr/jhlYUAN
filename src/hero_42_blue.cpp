#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h> 
#include <sstream> // for converting the command line parameter to integer
//#include <camera_info_manager/camera_info_manager.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>  //ROS已经内置了的串口包 

#include <hero/hero_17.h>
#include <hero/hero_17_mode.h>

#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>
#include "GxIAPI.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "DxImageProc.h"//me_data 
#include <iostream>

#include "armordetection42.h"

using namespace std;
using namespace cv;

cv::Mat m_image,test_image;
int bullet_number = 0;
long int loop_num = 1;
int mode = 1;
hero::hero_17 hero17;
hero::hero_17 hero42;
serial::Serial ser;
armordetection armor;
std::vector<float> IMU_data_;
std::vector<float> Motor_data_;
bool is_send = false;
int64_t m_width = 0;			///< 图像的宽
int64_t m_height = 0;			///< 图像的高
char* m_mono_buffer = NULL;		///< 黑白相机buffer
char* m_rgb_buffer = NULL;		///< 彩色相机buffer
bool b_is_color = true;		///< 相机是否支持彩色标志	*flag of pixel color filter 
int64_t m_roi_offset_x = 0;		///< 水平偏移量设置
int64_t m_roi_offset_y = 0;		///< 竖直偏移量设置
int64_t m_roi_width = 1280;		///< 感兴趣区域宽			
int64_t m_roi_height = 1024;		///< 感兴趣区域高
bool b_is_set_roi = false;		///< 是否设置roi标志

bool is_implemented = false;
int64_t m_pixel_color = 0;              ///< Bayer格式
char *m_rgb_image = NULL;

#define MEMORY_ALLOT_ERROR -1 

GX_DEV_HANDLE g_device = NULL;              //< 设备句柄
GX_FRAME_DATA g_frame_data = { 0 };         //< 采集图像参数
pthread_t g_acquire_thread = 0;             //< 采集线程ID
bool g_get_image = false;                   //< 采集线程是否结束的标志：true 运行；false 退出


//获取图像大小并申请图像数据空间
int PreForImage();

//释放资源
int UnPreForImage();

//采集线程函数
void *ProcGetImage(void* param);

//获取错误信息描述
void GetErrorString(GX_STATUS error_status);

void hero17Callback(const hero::hero_17::ConstPtr& sentry) 
{
    hero17.pitch = sentry -> pitch ;
    hero17.yaw = sentry -> yaw;
    hero17.x = sentry -> x;
    hero17.y = sentry -> y;
    hero17.z = sentry -> z;
    hero17.find_armor = sentry -> find_armor;
    ROS_INFO_STREAM("[ get sentry data!]"); 
}
int main(int argc, char** argv)
{
    //sensor_msgs::ImagePtr msg;
    //sensor_msgs::CameraInfo cam_info;
    ros::init(argc, argv, "hero_42_red");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("camera/detection", 10);
    ros::Publisher mode_pub = nh.advertise<hero::hero_17_mode>("hero/mode", 2);
    ros::Publisher hero_42_pub = nh.advertise<hero::hero_17>("hero/hero_42",1);
    ros::Subscriber hero_17_sub = nh.subscribe("hero/hero_17", 1, hero17Callback);
    //ros::Publisher pub2 = nh.advertise<sensor_msgs::CameraInfo>("back_camera/camera_info",100);
  
    armor.setup();
    armor.R_B = true;// Blue armor!
    
    //cv::namedWindow("view");
    //cv::namedWindow("src");    
    //cv::namedWindow("red");
    //cv::namedWindow("blue");
    //cv::startWindowThread();

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(230400); 
        serial::Timeout to = serial::Timeout::simpleTimeout(15); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    }
     //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    }  

    //大恒相机
    uid_t user = 0;
    user = geteuid();
    if(user != 0)
    {
        printf("\n");
        printf("Please run this application with 'sudo -E ./GxAcquireContinuous' or"
                              " Start with root !\n");
        printf("\n");
        return 0;
    }

    printf("\n");
    printf("-------------------------------------------------------------\n");
    printf("sample to show how to acquire image continuously.\n");
    #ifdef __x86_64__
    printf("version: 1.0.1605.8041\n");
    #elif __i386__
    printf("version: 1.0.1605.9041\n");
    #endif
    printf("-------------------------------------------------------------\n");
    printf("\n");

    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("Initializing......");
    printf("\n\n");
    usleep(500000);

    //API接口函数返回值 
    GX_STATUS status = GX_STATUS_SUCCESS;

    uint32_t device_num = 0;
    uint32_t ret = 0;
    GX_OPEN_PARAM open_param;

    //初始化设备打开参数，默认打开序号为1的设备
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = "1";

    //初始化库
    status = GXInitLib();
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return 0;
    }

    //获取枚举设备个数
    status = GXUpdateDeviceList(&device_num, 1000);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseLib();
        return 0;
    }

    if(device_num <= 0)
    {
        printf("<No device>\n");
        status = GXCloseLib();
        return 0;
    }
    else
    {
        //默认打开第1个设备
        status = GXOpenDevice(&open_param, &g_device);
        if(status == GX_STATUS_SUCCESS)
        {
            printf("<Open device success>\n");
	    b_is_set_roi = true;
	    //设置roi区域，设置时相机必须时停采状态
	    if(b_is_set_roi)
		{

		    status = GXSetInt(g_device,GX_INT_WIDTH,m_roi_width);
		    status = GXSetInt(g_device,GX_INT_HEIGHT,m_roi_height);
		    status = GXSetInt(g_device,GX_INT_OFFSET_X,m_roi_offset_x);
		    status = GXSetInt(g_device,GX_INT_OFFSET_Y,m_roi_offset_y);
		}
	    status =GXGetInt(g_device,GX_INT_WIDTH,&m_width);
	    status = GXGetInt(g_device,GX_INT_HEIGHT,&m_height);
	    status = GXIsImplemented(g_device,GX_ENUM_PIXEL_COLOR_FILTER,&b_is_color);
	    status = GXSetEnum(g_device,GX_ENUM_ACQUISITION_FRAME_RATE_MODE,GX_ACQUISITION_FRAME_RATE_MODE_ON);
	    status = GXSetFloat(g_device,GX_FLOAT_ACQUISITION_FRAME_RATE,90);
	    status = GXSetFloat(g_device,GX_FLOAT_EXPOSURE_TIME,1500);

        }
        else
        {
            printf("<Open device fail>\n");
            status = GXCloseLib();
            return 0;			
        }
    }

 /*   if(b_is_set_roi)
    {

    }else
    {
	if(b_is_color)
	{
	    m_image.create(m_height,m_width,CV_8UC3);
	}
    else{
	    m_image.create(m_height,m_width,CV_8UC1);
	}
    }  */

        if(b_is_color)
	{
	    m_image.create(m_height,m_width,CV_8UC3);
	}
    else{
	    m_image.create(m_height,m_width,CV_8UC1);
	}




    //设置采集模式为连续采集
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //设置触发开关为OFF
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
    //为采集做准备    
    ret = PreForImage();
    if(ret != 0)
    {
        printf("<Failed to prepare for acquire image>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //启动接收线程
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    bool run;
    if(ret != 0)
    {
        printf("<Failed to create the collection thread>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
    else
    {
        run = true;
        printf("<Create the collection thread successfully>\n");
    }

    ros::Rate loop_rate(250);

    while(run == true && nh.ok())
    {  
        if(!is_send)
        {
	    sensor_msgs::ImagePtr detectedimage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", armor.srcImage).toImageMsg();
            image_pub.publish(detectedimage);
	    
            hero_42_pub.publish(hero42);

	    hero::hero_17_mode result;
            result.mode = 1;
            mode_pub.publish(result);
            is_send = true;
	    //ROS_INFO_STREAM("send detected image! ");
                //cv::imshow("red",armor.rgbChannels[2]);
		//cv::imshow("blue",armor.rgbChannels[0]);

        	//cv::imshow("src",g_srcImage);
        	//cv::imshow("view",armor.srcImage);
        }	
	if(ser.available())
	{
    	    //ros::Time serial_time = ros::Time::now();             	
            //ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = ser.read(ser.available());
	    //float IMU_data;
        //    float Motor_data;
            //cout << "get serial" << result.data.size() << endl;
            unsigned char *p;
	    if(result.data[0] == -91 && result.data[1] == 90)
	    {
		p = (unsigned char *) & bullet_number;
		*p = result.data[2];
		*(p+1) = result.data[3];
		
		if(result.data[4] == 1)
		armor.B_S = true;
		else
		armor.B_S = false;
		//p = (unsigned char *) & mode;
                //*p = result.data[4];



		/*
		for(int i = 7; i > 3; i--)
                {
		    // std::cout << result.data[i] << std::endl;			
		    p = (unsigned char *) & IMU_data;
		    *(p+i-2) = result.data[i];             
	        }
                for(int i = 11; i > 7; i--)
                {
		    // std::cout << result.data[i] << std::endl;			
		    p = (unsigned char *) & IMU_data;
		    *(p+i-2) = result.data[i];             
	        }
                */
		
               	//std::cout << "1" << std::endl;
		//ROS_INFO_STREAM("bullet_number: " << bullet_number);
         	
		//ROS_INFO_STREAM("IMU_data: " << IMU_data);
		//ROS_INFO_STREAM("Motor_data: " << Motor_data);
		// if(IMU_data_.size() >= 100)
		// {
		//     IMU_data_.erase(IMU_data_.begin());		
		//     IMU_data_.push_back(IMU_data); 
        //     	}
		// else
		//     IMU_data_.push_back(IMU_data);
        // 	if(Motor_data_.size() >= 100)
		// {
		//     Motor_data_.erase(Motor_data_.begin());		
		//     Motor_data_.push_back(Motor_data); 
        // 	}
		// else
	    //  	    Motor_data_.push_back(Motor_data);
	     }
	//if(b_is_set_roi)
	//{
	//cam_info.height=m_roi_height;
	//cam_info.width=m_roi_width;
	//}else{
	//cam_info.height=1024;
	//cam_info.width=1280;
	//}
           
    
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    //cam_info.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    //cam_info.D.resize(5, 0.0);
    //cam_info.D[0]=-0.1279;
    //cam_info.D[1]=0.1206;
    //cam_info.D[2]=0;
    //cam_info.D[3]=0.0;
    //cam_info.D[4]=0.1899;
    // Give a reasonable default intrinsic camera matrix
    //cam_info.K = boost::assign::list_of(1070.05) (0.0) (660.4984)//(img->width/2.0)
    //       (0.0) (1069.202) (88.10422)
    //        (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    //cam_info.R = boost::assign::list_of (1.0) (0.0) (0.0)
    //        (0.0) (1.0) (0.0)
    //        (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    //cam_info.P = boost::assign::list_of (1.0) (0.0) (m_image.cols/2.0) (0.0)
    //        (0.0) (1.0) (m_image.rows/2.0) (0.0)
    //        (0.0) (0.0) (1.0) (0.0);


    //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_image).toImageMsg();
    //pub.publish(msg);
    //pub2.publish(cam_info);
	}



        ros::spinOnce();
        loop_rate.sleep();
    }
    //为停止采集做准备
    ret = UnPreForImage();
    if(ret != 0)
    {
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //关闭设备
    status = GXCloseDevice(g_device);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //释放库
    status = GXCloseLib();
    return 0;
}


/**
\brief 获取图像大小并申请图像数据空间
\return void
*/
//-------------------------------------------------
int PreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    int64_t payload_size = 0;
	
    status = GXGetInt(g_device, GX_INT_PAYLOAD_SIZE, &payload_size);
    printf("buffer size : %d \n",payload_size);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }
	
    g_frame_data.pImgBuf = malloc(payload_size);
    if(g_frame_data.pImgBuf == NULL)
    {
        printf("<Failed to allot memory>\n");
        return MEMORY_ALLOT_ERROR;
    }
    if(b_is_color)
	{
	    //m_rgb_buffer = new char[(size_t)m_width*m_height*3];
	    m_rgb_buffer = (char*)malloc(m_width*m_height*3);
	}
    else
	{
	    //m_mono_buffer = new char[(size_t)m_width*m_height];
	    m_mono_buffer = (char*)malloc(m_width*m_height);
	}
 
    return 0;
}

//-------------------------------------------------
/**
\brief 释放资源
\return void
*/
//-------------------------------------------------
int UnPreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ret = 0;
   
    //发送停采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_STOP);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }

    g_get_image = false;
    ret = pthread_join(g_acquire_thread,NULL);
    if(ret != 0)
    {
        printf("<Failed to release resources>\n");
        return ret;
    }
	

    //释放buffer
    if(g_frame_data.pImgBuf != NULL)
    {
        free(g_frame_data.pImgBuf);
        g_frame_data.pImgBuf = NULL;
    }

    if(b_is_color)
	{
	    if(m_rgb_buffer != NULL)
		{
	    	    free(m_rgb_buffer);
		    m_rgb_buffer = NULL;
		}
	    
	}
    else
	{
	    if(m_mono_buffer != NULL)
		{
	    	    free(m_mono_buffer);
		    m_mono_buffer = NULL;
		}
	    
	}

    return 0;
}
//-------------------------------------------------
/**
\brief 采集线程函数
\param pParam 线程传入参数
\return void*
*/
//-------------------------------------------------
void *ProcGetImage(void* pParam)
{
    GX_STATUS status = GX_STATUS_SUCCESS;

    //接收线程启动标志
    g_get_image = true;

    //发送开采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_START);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }

    while(g_get_image)
    {
        if(g_frame_data.pImgBuf == NULL)
        {
            continue;
        }

        status = GXGetImage(g_device, &g_frame_data, 100);
        if(status == GX_STATUS_SUCCESS)
        {
            if(g_frame_data.nStatus == 0)
            {
                //printf("<Successful acquisition 42mm : Width: %d Height: %d >\n", g_frame_data.nWidth, g_frame_data.nHeight);
		if(b_is_color)
		    {
			/// bayer转换
			DxRaw8toRGB24(g_frame_data.pImgBuf,m_rgb_buffer,g_frame_data.nWidth,g_frame_data.nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
			memcpy(m_image.data,m_rgb_buffer,g_frame_data.nWidth*g_frame_data.nHeight*3);
		    }
		else{
			memcpy(m_image.data,g_frame_data.pImgBuf,g_frame_data.nWidth*g_frame_data.nHeight);
		    }
	        
		//
		//cout <<"\n\n--------------------第\t" << loop_num++ << "帧--------------------"<<endl;
                ros::Time timeStart = ros::Time::now();
	    	cv::Mat g_srcImage = m_image;
            	armor.cut(g_srcImage);
	    	armor.Bright();
            	armor.BrighttoCanny();
            	armor.filter();

		    //cv::namedWindow("view");
    //cv::namedWindow("src");    
    //cv::namedWindow("red");
    //cv::namedWindow("blue");                
    //cv::startWindowThread();

		//cv::imshow("red",armor.rgbChannels[2]);
		//cv::imshow("blue",armor.rgbChannels[0]);

        	//cv::imshow("src",g_srcImage);
        	//cv::imshow("view",armor.srcImage);
            
	    	ros::Time timeFilte = ros::Time::now();
            	//cout << "filter time:" << (timeFilte - timeStart).toSec() * 1000<< endl;
            	
		hero42.pitch = armor.pnpresult.pitch * 1 * -1;
            	hero42.yaw = armor.pnpresult.yaw * 1 * 1;
            	hero42.x = armor.pnpresult.x;
            	hero42.y = armor.pnpresult.y;
            	hero42.z = armor.pnpresult.z;
            	hero42.find_armor = armor.pnpresult.find_armor;

		//serialdata.seq = msg->header.seq;
            	//serialdata.stamp = msg->header.stamp;
	    	//serialdata.pitch = armor.pnpresult.pitch * 1 * -1;
            	//serialdata.yaw = armor.pnpresult.yaw * 1 * -1;
            	//serialdata.x = armor.pnpresult.x;
            	//serialdata.y = armor.pnpresult.y;
            	//serialdata.z = armor.pnpresult.z;
            	//serialdata.find_armor = armor.pnpresult.find_armor; 
	    	//write_pub.publish(serialdata);
            	
		if(is_send)
            	    is_send = false;

		unsigned char *p;
      	    	std::vector<uint8_t> data;
	    	//报头
    	    	data.push_back(0xA5);
            	data.push_back(0x5A);
    	    	
                //42mm
                //pitch
		float pitch_add;
		
		if(armor.B_S)		
		{
		//pitch_add = 4.45;		
		pitch_add = -0.1964 * armor.pnpresult.z / 1000 * armor.pnpresult.z / 1000 + 3.0075 * armor.pnpresult.z / 1000 - 2.5643;
              cout << "pitch_add" << pitch_add << endl;
	}
		else
		{
		pitch_add = 0.0773 * armor.pnpresult.z / 1000 * armor.pnpresult.z / 1000 + 0.7659 * armor.pnpresult.z / 1000 + 2.6017;
              cout << "pitch_add" << pitch_add << endl;
		}
 		
		armor.pnpresult.pitch = -1 * armor.pnpresult.pitch + pitch_add;
  	    	
		p = (unsigned char *) &(armor.pnpresult.pitch);
    	    	data.push_back(*p);         //pitch的低8位
    	    	data.push_back(*(p+1));     //
    	    	data.push_back(*(p+2));     //
    	    	data.push_back(*(p+3));     //pitch的低8位
    	    	//yaw
		float yaw_add;

		if(armor.B_S)
		yaw_add = (armor.pnpresult.z - 3000) / 1000 * 0.2;
		else    	        
		yaw_add = (armor.pnpresult.z - 3000) / 1000 * 0.2;

		armor.pnpresult.yaw = armor.pnpresult.yaw - yaw_add;

    	    	p = (unsigned char *) &(armor.pnpresult.yaw);
   	    	data.push_back(*p);         //yaw的低8位
   	    	data.push_back(*(p+1));     //
  	    	data.push_back(*(p+2));     //
  	    	data.push_back(*(p+3));     //yaw的高8位
    	    	// //x
    	    	// p = (unsigned char *) &(armor.pnpresult.x);
   	    	// data.push_back(*p);         //x的低8位
  	    	// data.push_back(*(p+1));     //
   	    	// data.push_back(*(p+2));     //
  	    	// data.push_back(*(p+3));     //x的高8位
  	    	// //y
  	    	// p = (unsigned char *) &(armor.pnpresult.y);
 	    	// data.push_back(*p);         //y的低8位
	    	// data.push_back(*(p+1));     //
	    	// data.push_back(*(p+2));     //
	    	// data.push_back(*(p+3));     //y的高8位
	    	//z
	    	p = (unsigned char *) &(armor.pnpresult.z);
	    	data.push_back(*p);         //z的低8位
	    	data.push_back(*(p+1));     //
	    	data.push_back(*(p+2));     //
	    	data.push_back(*(p+3));     //z的高8位
	    	//是否找到
	    	p = (unsigned char *) &(armor.pnpresult.find_armor);
	    	data.push_back(*p);         //
 		
	    	//bullet_number
	    	p = (unsigned char *) &(bullet_number);
	    	data.push_back(*p);         //bullet_number的低8位
	    	data.push_back(*(p+1));     //
		
                //17mm
    	    	//pitch
    	    	p = (unsigned char *) &(hero17.pitch);
    	    	data.push_back(*p);         //pitch的低8位
    	    	data.push_back(*(p+1));     //
    	    	data.push_back(*(p+2));     //
    	    	data.push_back(*(p+3));     //pitch的高8位
    	    	//yaw
    	    	//float yaw_hero =  0.1 * serialdata -> yaw;
    	    	p = (unsigned char *) &(hero17.yaw);
   	    	data.push_back(*p);         //yaw的低8位
   	    	data.push_back(*(p+1));     //
  	    	data.push_back(*(p+2));     //
  	    	data.push_back(*(p+3));     //yaw的高8位
    	    	// //x
    	    	// p = (unsigned char *) &(hero17.x);
   	    	// data.push_back(*p);         //x的低8位
  	    	// data.push_back(*(p+1));     //
   	    	// data.push_back(*(p+2));     //
  	    	// data.push_back(*(p+3));     //x的高8位
  	    	// //y
  	    	// p = (unsigned char *) &(hero17.y);
 	    	// data.push_back(*p);         //y的低8位
	    	// data.push_back(*(p+1));     //
	    	// data.push_back(*(p+2));     //
	    	// data.push_back(*(p+3));     //y的高8位
	    	//z
	    	p = (unsigned char *) &(hero17.z);
	    	data.push_back(*p);         //z的低8位
	    	data.push_back(*(p+1));     //
	    	data.push_back(*(p+2));     //
	    	data.push_back(*(p+3));     //z的高8位
	    	//是否找到
	    	p = (unsigned char *) &(hero17.find_armor);
	    	data.push_back(*p);         //
	    	//报尾
	    	data.push_back(0xff);
	    	ser.write(data);
	    	ros::Time timeEnd = ros::Time::now();
            	//cout << "detecting armor in this frame takes\t" << (timeEnd - timeStart).toSec() * 1000 << "\t\tms!\n";
            	//cout << "------------------------------------------------\n" << endl;
            }
        }
    }
}

//----------------------------------------------------------------------------------
/**
\brief  获取错误信息描述
\param  emErrorStatus  错误码

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS error_status)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS status = GX_STATUS_SUCCESS;

    // 获取错误描述信息长度
    status = GXGetLastError(&error_status, NULL, &size);
    if(status != GX_STATUS_SUCCESS)
    {
           GetErrorString(status);
           return;
    }

    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }

    // 获取错误信息描述
    status = GXGetLastError(&error_status, error_info, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        printf("<GXGetLastError call fail>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // 释放资源
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

