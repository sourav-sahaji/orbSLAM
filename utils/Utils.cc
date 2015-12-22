/*
 * Utils.cc
 *
 *  Created on: 22 May 2015
 *      Author: yasir
 */

#include "Utils.h"


namespace utils
{
namespace image
{
	cv::Mat toMat(const ORB_Types::Image& lcm_image)
	{
		cv::Mat im(
				lcm_image.height,
				lcm_image.width,
				lcm_image.pixelformat,
				(unsigned char*)&lcm_image.data[0]);
		return im;
	}

	ORB_Types::Image toLCM(const cv::Mat& im, double timestamp)
	{


		ORB_Types::Image imOut;
		imOut.pixelformat 		= im.type();
		imOut.width 			= im.cols;
		imOut.height 			= im.rows;
		imOut.row_stride 		= im.step;
		imOut.utime 			= timestamp	;
		imOut.size 				= int(im.dataend - im.datastart);
		imOut.data.resize( imOut.size );

		memcpy((void*)&imOut.data[0],(void*)im.data,imOut.size);
		return imOut;
	}


	ImageDisplay::ImageDisplay(): name("imDisplay"), lcmInstance(lcm::LCM()){ cv::namedWindow(name);}

	ImageDisplay::ImageDisplay(const std::string& windowName) : name(windowName),lcmInstance(lcm::LCM()){ cv::namedWindow(name); }

	ImageDisplay::~ImageDisplay(){}

	void ImageDisplay::Receiver(
			const lcm::ReceiveBuffer* rbuf,
			const std::string& chan,
			const ORB_Types::Image* msg)
	{
		std::cerr<<"Received :"<<msg->utime<<std::endl;
		cv::Mat im = toMat(*msg);
		if(im.size[0] == 0)
		{
			std::cerr<<"Couldn't process image!"<<std::endl;
		}
		cv::imshow(name, im);
		cv::updateWindow(name);
		cv::waitKey(5);
	}

	void ImageDisplay::ListenTo(const char* topic)
	{
		lcmInstance.subscribe(topic,&ImageDisplay::Receiver, this);
		while(0 == lcmInstance.handle() and lcmInstance.good());
	}

	ImagePublisher:: ImagePublisher(): topic("/image"), lcmInstance(lcm::LCM()){}
	ImagePublisher:: ImagePublisher(const std::string& topic): topic(topic), lcmInstance(lcm::LCM()){}
	void ImagePublisher::Publish(cv::Mat& image, int timestamp)
	{
		ORB_Types::Image imLCM = toLCM(image,timestamp);  // TODO: Put the correct system time here
		lcmInstance.publish(topic.c_str(),&imLCM);
	}

	RateLimitedImagePublisher::RateLimitedImagePublisher(): ImagePublisher(), rate(30){}
	RateLimitedImagePublisher::RateLimitedImagePublisher(const float rate, const std::string& topic): ImagePublisher(topic), rate(rate){}

	void RateLimitedImagePublisher::readFileAndPublish(const char* filename, bool show, int start)
	{
		std::ifstream in(filename);
		char imageName[1024];
		int count = 0;

		while(in.good() and in.peek()!=in.eof())
		{
			in>>imageName;
			if(count < start )
			{
				count++;
				continue; // Would read until count == start to publish
			}


			cv::Mat im = cv::imread(imageName,CV_LOAD_IMAGE_UNCHANGED);
			std::cerr<<"INFO: [ImagePublisher] :"<<topic<<" "<<imageName<<std::endl;
			Publish(im);
			if(show)
			{
				cv::imshow("image", im);
				cv::waitKey(3);
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(int(1./rate*1000)));
		}
	}


}
}
