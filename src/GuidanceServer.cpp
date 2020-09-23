/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic


ros::Publisher left_image_pub;
ros::Publisher right_image_pub;

ros::Publisher caminfo_left_pub;
ros::Publisher caminfo_right_pub;

ros::Publisher obstacle_distance_pub;

using namespace cv;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

sensor_msgs::CameraInfo cameraInfo_left;
sensor_msgs::CameraInfo cameraInfo_right;


char        	key       = 0;

DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat				g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat				g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat				depth8(HEIGHT, WIDTH, CV_8UC1);

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
	const char* s = 0;
	static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value){
		PROCESS_VAL(e_OK);     
		PROCESS_VAL(e_load_libusb_err);     
		PROCESS_VAL(e_sdk_not_inited);
		PROCESS_VAL(e_disparity_not_allowed);
		PROCESS_VAL(e_image_frequency_not_allowed);
		PROCESS_VAL(e_config_not_ready);
		PROCESS_VAL(e_online_flag_not_ready);
		PROCESS_VAL(e_stereo_cali_not_ready);
		PROCESS_VAL(e_libusb_io_err);
		PROCESS_VAL(e_timeout);
	default:
		strcpy(str, "Unknown error");
		s = str;
		break;
	}
#undef PROCESS_VAL

	return out << s;
}

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();



    /* image data */
    if (e_image == data_type && NULL != content)
    {        
        image_data* data = (image_data*)content;

		if ( data->m_greyscale_image_left[e_vbus1] &&data->m_greyscale_image_right[e_vbus1]){
			memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[e_vbus1], IMAGE_SIZE);
			// publish left greyscale image
			cv_bridge::CvImage left_8;
			g_greyscale_image_left.copyTo(left_8.image);
			left_8.header.frame_id  = cameraInfo_left.header.frame_id="guidance";

			left_8.encoding		= sensor_msgs::image_encodings::MONO8;

			memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[e_vbus1], IMAGE_SIZE);
			// publish right greyscale image
			cv_bridge::CvImage right_8;
			g_greyscale_image_right.copyTo(right_8.image);
			right_8.header.frame_id  = cameraInfo_right.header.frame_id="guidance";

			right_8.encoding  	 = sensor_msgs::image_encodings::MONO8;

            right_8.header.stamp	 = cameraInfo_right.header.stamp=left_8.header.stamp	= cameraInfo_left.header.stamp=ros::Time::now();
            left_image_pub.publish(left_8.toImageMsg());
            caminfo_left_pub.publish(cameraInfo_left);
            right_image_pub.publish(right_8.toImageMsg());
            caminfo_right_pub.publish(cameraInfo_right);
		}


    }


    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
		// publish obstacle distance
		sensor_msgs::LaserScan g_oa;
		g_oa.ranges.resize(CAMERA_PAIR_NUM);
		g_oa.header.frame_id = "guidance";
		g_oa.header.stamp    = ros::Time::now();
		for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
			g_oa.ranges[i] = 0.01f * oa->distance[i];
		obstacle_distance_pub.publish(g_oa);
	}


    g_lock.leave();
    g_event.set_event();

    return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

int main(int argc, char** argv)
{
	
    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;

    left_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/left/image_raw",1);
    right_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/right/image_raw",1);

    caminfo_left_pub =
            my_node.advertise<sensor_msgs::CameraInfo>("/guidance/left/camera_info", 1);

    caminfo_right_pub =
            my_node.advertise<sensor_msgs::CameraInfo>("/guidance/right/camera_info", 1);



    obstacle_distance_pub	= my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);



    cameraInfo_left.height=320;
    cameraInfo_left.width=480;
    cameraInfo_left.distortion_model="plumb_bob";
    cameraInfo_left.D={0.0080066,-0.02026311,0.00149403,0.00023621,0};
    cameraInfo_left.K={239.04790404,0,157.26877462,0,240.0469,129.87152724,0,0,1};
    cameraInfo_left.R={1,0,0,0,1,0,0,0,1};

    cameraInfo_left.P={239.04790404,0       ,157.26877462,0,
                       0           ,240.0469,129.87152724,0,
                       0           ,0       ,      1    ,0};

    cameraInfo_right.height=320;
    cameraInfo_right.width=480;
    cameraInfo_right.distortion_model="plumb_bob";
    cameraInfo_right.D={-0.0200949,0.04735013,0.00092343,-0.00098452,0};
    cameraInfo_right.K={240.37836821,0,156.81987377,
                        0,241.22439699,128.33110269,
                        0,0,1};

    cameraInfo_right.R={1,0.00275,-0.00112,
                        -0.0028,1,-0.00441,
                        0.00112,-0.00441,1};
    cameraInfo_right.P={239.04790404,0       ,157.26877462,-239.04790404*0.15,
                        0,241.22439699,128.33110269,0,
                        0,0,1,0
    };

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	RETURN_IF_ERR(err_code);
    std::cout<<"Sensor online status: ";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
        std::cout<<online_status[i]<<" ";
    std::cout<<std::endl;

	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
	RETURN_IF_ERR(err_code);
    std::cout<<"cu\tcv\tfocal\tbaseline\n";
	for (int i=0; i<CAMERA_PAIR_NUM; i++)
	{
        std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
	}
	
    /* select data */
    err_code = select_greyscale_image(e_vbus1, true);
	RETURN_IF_ERR(err_code);
    err_code = select_greyscale_image(e_vbus1, false);
	RETURN_IF_ERR(err_code);


    select_obstacle_distance();

    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);
	
	std::cout << "start_transfer" << std::endl;
    set_image_frequecy(e_frequecy_20);

	while (ros::ok())
	{
		g_event.wait_event();
        ros::spinOnce();
	}

	/* release data transfer */
	err_code = stop_transfer();
	RETURN_IF_ERR(err_code);
	//make sure the ack packet from GUIDANCE is received
	sleep(1);
	std::cout << "release_transfer" << std::endl;
	err_code = release_transfer();
	RETURN_IF_ERR(err_code);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
