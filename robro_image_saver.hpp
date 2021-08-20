/**
 * @file robro_image_saver.hpp
 * @author Harshit Sureka (harshit.sureka@robrosystems.com)
 * @brief Simple Header to Save images at a particular location, maintain queue and resize, if reqd.
 * @version 1.1
 * @date 2021-06-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
// Including Necessary Libraries.
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class RobroImageSaver
{
    /**
     * @brief Simple class to save images
     * @TODO: Need to add threading, overwrite protection etc.
     */

public:
    RobroImageSaver(std::string, std::string, std::string);
    ~RobroImageSaver();

    std::string base_path;
    std::string image_prefix;
    std::string image_suffix;

    int image_counter;
    int crop_row, crop_col, crop_width, crop_height;
    int resize_width, resize_height;
    bool crop_enabled;
    bool resize_enabled;
    bool kill_thread;
    std::queue<cv::Mat> image_queue;

    void imageSaverThread();

    void save(cv::Mat img);

    /*
     * Helper functions to crop and resize. Remember, CROP happens before RESIZE.
     * RESIZE always preserves aspect ratio
     */
    void setCropParameters(int row, int col, int width, int height);
    void setResizeParameters(int width, int height);
};

/**
 * @brief Thread to write images
 * 
 */
void RobroImageSaver::imageSaverThread()
{
	while(!kill_thread)
	{
		if(image_queue.empty())
		{
			sleep(1);
		}
		else
		{
	    		cv::imwrite(base_path + image_prefix + std::to_string(image_counter++) + image_suffix, image_queue.front());
			image_queue.pop();
		}
	}
}
/**
 * @brief Construct a new Robro Image Saver object
 * 
 * @param _base_path 
 * @param _prefix 
 * @param _sufffix 
 */
RobroImageSaver::RobroImageSaver(std::string _base_path, std::string _prefix, std::string _suffix)
{
    base_path = _base_path;
    image_prefix = _prefix;
    image_suffix = _suffix;
    crop_enabled = false;
    resize_enabled = false;
    image_counter = 0;
    kill_thread = false;
    std::thread image_saver_thread(&RobroImageSaver::imageSaverThread, this);
    image_saver_thread.detach();

    if(base_path.back() != '/')
    {
        base_path = base_path + '/';
    }
}

/**
 * @brief Set cropping parameters. Cropping happens before resizing, if enabled.
 * 
 * @param row
 * @param col
 * @param width
 * @param height
 */
void RobroImageSaver::setCropParameters(int row, int col, int width, int height)
{
	if(width <= 0 || height <= 0)
	{
		std::cout << "[ERROR] Can't set crop width or height <= 0\n";
	}
	else
	{
		crop_row = row;
		crop_col = col;
		crop_width = width;
		crop_height = height;
	}
}

/**
 * @brief Set resizing to be enabled or disabled
 * Crop, if enabled, happens before resizing. Then, the cropped image is resized.
 * 
 * @param width
 * @param height
 */
void RobroImageSaver::setResizeParameters(int width, int height)
{
	if(width <= 0 || height <= 0)
	{
		std::cout << "[ERROR] Can't set resize width or height <= 0\n";
	}
	else
	{
		resize_width = width;
		resize_height = height;
	}
}


/**
 * @brief Crop, Resize and Add image to save queue
 * 
 * @param img 
 */
void RobroImageSaver::save(cv::Mat img)
{
    if(crop_enabled)
    {
	    cv::Rect crop_rect(crop_col, crop_row, crop_width, crop_height);
	    img = img(crop_rect);
    }

    if(resize_enabled)
    {
	    cv::resize(img, img, cv::Size(resize_width, resize_height), 0, 0, cv::INTER_LINEAR);
    }

    image_queue.push(img);
}

/**
 * @brief Destroy the Robro Image Saver object
 * 
 */
RobroImageSaver::~RobroImageSaver()
{
	kill_thread = true;
}
