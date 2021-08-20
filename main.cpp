/**
 * @file main.cpp 
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Main for Weaving Inspection
 * @version 1.0
 * @date 2021-05-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
// Including Necessary Libraries.
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

#include "inference/inference/robro_darkhelp/robro_darkhelp.hpp"
#include "camera/cpp/hikvisionCamera/hikCamera.hpp"
//#include "camera/pylonCamera/pylonCamera.hpp"
#include "camera/cpp/cameraSimulation/cameraSimulation.hpp"
#include "image_saver/robro_image_saver.hpp"
#include "jetson_nano_gpio/JetsonNanoGPIO.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ctime>
#include <iostream>
#include <thread>
#include <sstream>
#include <string>

class WeavingInspection
{
    /**
	 * @brief Main class that handles files inspection application
	 * 
	 */
public:
    WeavingInspection();
    ~WeavingInspection();

    bool SIMULATION_MODE;

    std::string cfg_file_path, weights_file_path, names_file_path;
    std::string image_path_suffix, image_path_prefix;
    std::string simulation_images_path;
    std::string defect_type_count_array_topic;

    float threshold_value;

    std::atomic<bool> continuous_grabbing_on;

    bool save_image_on;

    std::string image_publish_topic, current_result_publish_topic;
    std::string ok_count_publish_topic, ng_count_publish_topic, total_count_publish_topic, machine_status_publish_topic;
    std::string start_button_subscribe_topic, stop_button_subscribe_topic;
    std::string save_image_flag_subscribe_topic, threshold_value_subscribe_topic, pause_button_subscribe_topic, previous_image1_publish_topic, previous_image2_publish_topic, previous_image3_publish_topic;
    std::string save_image_base_path;

    int32_t ok_count, ng_count, total_count, hole_count, spot_count;
    std_msgs::String ok_count_publish_msg, ng_count_publish_msg, total_count_publish_msg;

    image_transport::Publisher image_publisher, previous_image1_publisher, previous_image2_publisher, previous_image3_publisher;
    ros::Publisher current_result_publisher;
    ros::Publisher no_of_defect_publisher, machine_status_publisher;
    ros::Publisher ok_count_publisher, ng_count_publisher, total_count_publisher, defect_type_count_publisher;

    ros::Subscriber start_button_subscriber, stop_button_subscriber, save_image_flag_subscriber, pause_button_subscriber;
    ros::Subscriber threshold_value_subscriber;

    RobroDarkHelp *inference;
    RobroCameraAbstractClass *camera;
    RobroImageSaver *image_saver;
    JetsonGPIO *initial;

    int resize_width, resize_height, crop_width, crop_height, crop_col, crop_row;
    bool resize_image_enabled, crop_image_enabled;

    void processAndPublishImage(cv::Mat);
    void startContinuousGrab(const std_msgs::Empty &msg);
    void stopContinuousGrab(const std_msgs::Empty &msg);
    void pauseCallback(const std_msgs::Empty &msg);
    void saveImageFlagCallback(const std_msgs::Bool &msg);
    void thresholdCallback(const std_msgs::Float32 &msg);
    void buzzer();
};

/**
 * @brief Construct a new FilesInspection Application:: FilesInspection Application object
 * 
 */
WeavingInspection::WeavingInspection()
{

    ros::NodeHandle n;

    // Read parameters with default value assigned

    ROS_INFO("Loading basic params...");

    ros::param::get("~simulation_mode_on", SIMULATION_MODE);
    ros::param::get("~simulation_images_path", simulation_images_path);
    ros::param::get("~simulation_images_path_prefix", image_path_prefix);
    ros::param::get("~simulation_images_path_suffix", image_path_suffix);

    ros::param::get("~resize_width",resize_width);
    ros::param::get("~resize_height",resize_height);
    ros::param::get("~resize_image_enabled",resize_image_enabled);
    ros::param::get("~crop_image_enabled",crop_image_enabled);
    ros::param::get("~crop_col",crop_col);
    ros::param::get("~crop_row",crop_row);
    ros::param::get("~crop_width",crop_width);
    ros::param::get("~crop_height",crop_height);

    ros::param::get("~cfg_file_path", cfg_file_path);
    ros::param::get("~names_file_path", names_file_path);
    ros::param::get("~weights_file_path", weights_file_path);
    ros::param::get("~theshold_value", threshold_value);

    ros::param::get("~image_transport_topic_name", image_publish_topic);
    ros::param::get("~previous_image1_topic_name", previous_image1_publish_topic);
    ros::param::get("~previous_image2_topic_name", previous_image2_publish_topic);
    ros::param::get("~previous_image3_topic_name", previous_image3_publish_topic);
    ros::param::get("~current_result_publish_topic", current_result_publish_topic);
    ros::param::get("~ok_count_topic_name", ok_count_publish_topic);
    ros::param::get("~ng_count_topic_name", ng_count_publish_topic);
    ros::param::get("~total_count_topic_name", total_count_publish_topic);
    ros::param::get("~machine_status_topic_name", machine_status_publish_topic);
    ros::param::get("~defect_type_count_array_topic_name", defect_type_count_array_topic);
    

    ROS_INFO("Loading topic names...");

    ros::param::get("~start_button_topic_name", start_button_subscribe_topic);
    ros::param::get("~stop_button_topic_name", stop_button_subscribe_topic);
    ros::param::get("~pause_button_topic_name", pause_button_subscribe_topic);
    ros::param::get("~save_image_flag_topic_name", save_image_flag_subscribe_topic);
    ros::param::get("~threshold_value_topic_name", threshold_value_subscribe_topic);

    ros::param::get("~save_image_base_path", save_image_base_path);

    image_transport::ImageTransport it(n);
    unsigned int f_outputs[3] = {149, 200, 79};
    unsigned int f_inputs[2] = {194, 38};
    int output = f_outputs[2];
    bool f_debug_mode = false;
    initial = new JetsonGPIO(f_inputs, f_outputs, f_debug_mode);

    /***
     * PUBLISHERS
     */

    current_result_publisher = n.advertise<std_msgs::String>(current_result_publish_topic, 1);
    ok_count_publisher = n.advertise<std_msgs::String>(ok_count_publish_topic, 1);
    ng_count_publisher = n.advertise<std_msgs::String>(ng_count_publish_topic, 1);
    total_count_publisher = n.advertise<std_msgs::String>(total_count_publish_topic, 1);
    machine_status_publisher = n.advertise<std_msgs::String>(machine_status_publish_topic, 1);
    defect_type_count_publisher = n.advertise<std_msgs::Int32MultiArray>(defect_type_count_array_topic, 10);

    image_publisher = it.advertise(image_publish_topic, 1);
    previous_image1_publisher = it.advertise(previous_image1_publish_topic, 1);
    previous_image2_publisher = it.advertise(previous_image2_publish_topic, 1);
    previous_image3_publisher = it.advertise(previous_image3_publish_topic, 1);

    ROS_INFO("Setting Publishers done...");

    /***
     * SUBSCRIBERS
     */
    start_button_subscriber = n.subscribe(start_button_subscribe_topic, 1, &WeavingInspection::startContinuousGrab, this);
    stop_button_subscriber = n.subscribe(stop_button_subscribe_topic, 1, &WeavingInspection::stopContinuousGrab, this);
    pause_button_subscriber = n.subscribe(pause_button_subscribe_topic, 1, &WeavingInspection::pauseCallback, this);

    save_image_flag_subscriber = n.subscribe(save_image_flag_subscribe_topic, 1, &WeavingInspection::saveImageFlagCallback, this);
    threshold_value_subscriber = n.subscribe(threshold_value_subscribe_topic, 1, &WeavingInspection::thresholdCallback, this);

    std::cout << "Simulation Mode: " << SIMULATION_MODE << std::endl;

    image_saver = new RobroImageSaver(save_image_base_path, "image_", ".png");

    /*
     * We are doing resizing and cropping in this main code itself. No need to do again in saver.
     * It is important to do in this code since we want to view the same image we are saving.
     *
    if(resize_image_enabled)
    {
    	image_saver->resize_enabled = true;
    	image_saver->setResizeParameters(resize_width, resize_height);
    }

    if(crop_image_enabled)
    {
	    image_saver->crop_enabled = true;
	    image_saver->setCropParameters(crop_row, crop_col, crop_width, crop_height);
    }
    */

    if (SIMULATION_MODE)
    {
        camera = new RobroCameraSimulation(simulation_images_path, image_path_prefix, image_path_suffix);
        camera->m_continous_trigger_frequency = 10; // Send 10 images per sec
    }
    else
    {
        camera = new HikCamera();
    }

    continuous_grabbing_on = false;
    save_image_on = false;

    //inference = new RobroDarkHelp(cfg_file_path, weights_file_path, names_file_path, threshold_value);

    ok_count = ng_count = total_count = hole_count = spot_count = 0;
    //chisel_count = chopping_count = uneven_cut_count = fine_cut_count = 0;
}

/**
 * @brief  save image flag call back
 * 
 */
void WeavingInspection::saveImageFlagCallback(const std_msgs::Bool &msg)
{
    save_image_on = msg.data;
    ROS_INFO("Image Saving Turned: %s", (msg.data ? "ON" : "OFF"));
}

/**
 * @brief Stop the image transfer
 * 
 */
void WeavingInspection::stopContinuousGrab(const std_msgs::Empty &msg)
{
    std::cout << "Stopping Grabbing\n";
    continuous_grabbing_on = false;
    std_msgs::String sending_msg;
    sending_msg.data = "Ideal";
    machine_status_publisher.publish(sending_msg);
    ng_count = ok_count = total_count = hole_count = spot_count = 0;
    initial->setOutputOff(79);
    //chisel_count = chopping_count = uneven_cut_count = fine_cut_count = 0;
}

/**
 * @brief pause
 * 
 * @param msg 
 */
void WeavingInspection::pauseCallback(const std_msgs::Empty &msg)
{
    continuous_grabbing_on = false;
    std_msgs::String sending_msg;
    sending_msg.data = "Paused";
    machine_status_publisher.publish(sending_msg);
}

/**
 * @brief Start the Image Capture
 * 
 */
void WeavingInspection::buzzer()
{ 
  initial->setOutputOn(79);
  sleep(1);
  initial->setOutputOff(79);
}

void WeavingInspection::startContinuousGrab(const std_msgs::Empty &msg)
{ 
    
    
    
    std::cout << "Starting Grabbing\n";
    continuous_grabbing_on = true;
    std_msgs::String sending_msg;
    sending_msg.data = "Running";
    machine_status_publisher.publish(sending_msg);
    
    
}

/**
 * @brief Change the current threshold value of network.
 * 
 */
void WeavingInspection::thresholdCallback(const std_msgs::Float32 &msg)
{
    //ROS_INFO("Threshold changing from %f to %f\n", inference->darkhelp->threshold, msg.data);
    //inference->darkhelp->threshold = msg.data;
}

/**
 * @brief Destroy the FilesInspection Application:: FilesInspection Application object
 * 
 */
WeavingInspection::~WeavingInspection()
{
    delete camera;
    delete initial;
}

/**
 * @brief  Predict on the image. Publish it out.
 * 
 * @param img Image to be predicted on
 */
void WeavingInspection::processAndPublishImage(cv::Mat img)
{
    // Do Predictions on the image.
    //inference->predict(img, true);
    //std::cout << inference->predictions << std::endl;
 
    //std_msgs::String current_result_msg;

    //if(!inference->predictions.empty())
    //{
        //TODO: Better logic for deciding current class
        //current_result_msg.data = inference->predictions[0].name;
        //if (current_result_msg.data.find("hole") != std::string::npos) 
        //{
            //ng_count++;
           // hole_count++;
	   // current_result_msg.data = "HOLE";
	   // std::thread thread_buzzer (&WeavingInspection::buzzer, this);
	   // thread_buzzer.detach();
            //buzzer();
            //initial->setOutputOn(79);
            //std::cout << "Found Chisel\n";
        //}
        //else if(current_result_msg.data.find("spot") != std::string::npos)
        //{
           // ng_count++;
            //spot_count++;
            //current_result_msg.data = "SPOT";
	    //std::thread thread_buzzer (&WeavingInspection::buzzer, this);
	    //thread_buzzer.detach();
            //buzzer();
            //initial->setOutputOn(79);
       // }
        
   // }
   // else
   // {
        //current_result_msg.data = "OK";
    //ok_count++;
        //initial->setOutputOff(79);
   // }

   //total_count = ok_count + ng_count;

    //try //PUBLISHING THINGS
    //{
        //IMAGE
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_publisher.publish(msg);
    std::cout<<"Image Published"<<std::endl;
    ok_count++;
    total_count = ok_count + ng_count;
    ok_count_publish_msg.data = std::to_string(ok_count).c_str();
    total_count_publish_msg.data = std::to_string(total_count).c_str();
    ng_count_publish_msg.data = std::to_string(ng_count).c_str();
    ok_count_publisher.publish(ok_count_publish_msg);
    ng_count_publisher.publish(ng_count_publish_msg);
    total_count_publisher.publish(total_count_publish_msg);
        //current_result_publisher.publish(current_result_msg);

        //FOR CHART
        //if(total_count % 5 == 0)
        //{
           // std_msgs::Int32MultiArray array;
            //array.data.clear();
           // array.data.push_back(ok_count);
           // array.data.push_back(spot_count);
           // array.data.push_back(hole_count);
           // defect_type_count_publisher.publish(array);
       // }

        //OK, NG, Total
       // ok_count_publish_msg.data = std::to_string(ok_count).c_str();
        //ng_count_publish_msg.data = std::to_string(ng_count).c_str();
       // total_count_publish_msg.data = std::to_string(total_count).c_str();

       // ok_count_publisher.publish(ok_count_publish_msg);
       // ng_count_publisher.publish(ng_count_publish_msg);
       // total_count_publisher.publish(total_count_publish_msg);
   // }
    //catch (const std::exception &e)
    //{
    //    std::cerr << e.what() << '\n';
   // }
}

/**
 * @brief Main Function which creates the objects
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "weaving_inspection_node");

    ROS_INFO("In main...");

    WeavingInspection weaving_inspection;

    ROS_INFO("created object...");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Spinner Started...");

    ros::Rate rate(100);
    cv::Mat image;
    int save_image_counter = 0;
                                                
    while (ros::ok())
    {
        if (weaving_inspection.continuous_grabbing_on)
        {
            weaving_inspection.initial->interruptCheck(194);
	    ROS_INFO("Trigger Occured!");
            if (weaving_inspection.camera->grabNextImage(image) && weaving_inspection.continuous_grabbing_on)
            {
		//cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

		if(weaving_inspection.crop_image_enabled)
		{
			cv::Rect crop_rect(weaving_inspection.crop_col, weaving_inspection.crop_row, weaving_inspection.crop_width, weaving_inspection.crop_height);
			image = image(crop_rect);
		}

		if(weaving_inspection.resize_image_enabled)
		{
			cv::resize(image, image, cv::Size(weaving_inspection.resize_width, weaving_inspection.resize_height), 0, 0, cv::INTER_LINEAR);
		}

		weaving_inspection.processAndPublishImage(image);
                
		if (weaving_inspection.save_image_on)
                {
                    weaving_inspection.image_saver->save(image);
                    std::cout << "Saving images\n";
		}
                //weaving_inspection.initial->Control();
            }
            else
            {
                std::cout << "[ERROR] Grabbing Failed\n";
            }
        }
        rate.sleep();
    }
    return 0;
}
