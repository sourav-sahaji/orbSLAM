/*
 * Visualizer.cc
 *
 *  Created on: 15 Jun 2015
 *      Author: yasir
 */

#include "Visualizer.h"


namespace Visualizer{



void MapVisualizer::MapReceiver(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const ORB_Types::Map* msg)
{
	//std::cerr<<"GOT : "<<chan<<std::endl;

	std::lock_guard<std::mutex> lock(mapMutex);
	map = *msg;
	mapUpdated = true;
}

void MapVisualizer:: ImageReceiver(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const ORB_Types::Image* msg)
{
	//std::cerr<<"GOT : "<<chan<<std::endl;
	std::lock_guard<std::mutex> lock(imageMutex);
	cv::Mat im  = utils::image::toMat(*msg);
	im.convertTo(trackedImage,CV_8U);
	imageUpdated = true;
}

void MapVisualizer:: PoseReceiver(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const ORB_Types::PoseSE3* msg)
{
	//std::cerr<<"GOT : "<<chan<<std::endl;
	std::lock_guard<std::mutex> lock(poseMutex);
	currentPose = *msg;
	poseUpdated = true;
}

void MapVisualizer::printHelp()
{
	std::cerr<<"Key mappings for the viewer "<<std::endl
			<<"+/- 	: Increase/ Decrease Point size "<<std::endl
			<<"f	: free the camera"<<std::endl
			<<"m	: modes of camera overhead/current view"<<std::endl
			<<"s 	: STOP the viewer "<<std::endl;
}

void MapVisualizer::keyboardCallback(const viz::KeyboardEvent& e)
{
	switch (e.code)
	{
	case '+':
	{	pointSize++;
	break;
	}
	case '-':
	{ 	if(pointSize > 1)
		pointSize--;
	break;
	}
	case 's':
	{ 	quit = true;
	break;
	}
	case 'm':
	{
		updateCamera = true;
		if(CAM_MODE == CAM_FREE) { CAM_MODE = CAM_OVERHEAD; break;}
		if (CAM_MODE == CAM_OVERHEAD){ CAM_MODE = CAM_CURRENT_POSE; break; }
		if (CAM_MODE == CAM_CURRENT_POSE) { CAM_MODE = CAM_OVERHEAD; break; }
		break;
	}
	case 'f':
	{
		updateCamera = true;
		CAM_MODE = CAM_FREE;
		break;
	}
	default:
	{
		printHelp();
		break;
	}
	}
}
bool MapVisualizer::UpdateLoop()
{
	bool showUpdatedMap = false;

	ORB_Types::PoseSE3 poseNow = currentPose;

	while(!quit)
	{

		{ // scoped lock
			std::lock_guard<std::mutex> lock(mapMutex);
			if(mapUpdated)
			{
				_local = map;
				mapUpdated = false;
				showUpdatedMap = true;
			}
		}

		{ // scoped lock
			std::lock_guard<std::mutex> lock(imageMutex);
			if(imageUpdated)
			{
				windowImage->setWindowSize(trackedImage.size());
				windowImage->showImage(trackedImage,trackedImage.size());
				imageUpdated = false;
				windowImage->spinOnce(1,true);
			}
		}

		{ //scoped lock
			std::lock_guard<std::mutex> lock(poseMutex);
			if(poseUpdated)
			{
				poseNow = currentPose;
				poseUpdated = false;
			}
		}

		if(showUpdatedMap)
		{

			showCloud(_local.points,"mapPoints",cv::viz::Color::green());
			showCloud(_local.referencePoints,"refPoints",cv::viz::Color::red());

			std::vector<cv::Vec3d> keyFrames;
			for (auto &p: _local.poses) keyFrames.push_back(cv::Vec3d(p.translation.x, p.translation.y, p.translation.z));
			if(!keyFrames.empty())
			{

				window->showWidget("keyFrames",viz::WCloud(keyFrames,cv::viz::Color::blue()));
				window->setRenderingProperty("keyFrames",viz::POINT_SIZE,8);

			}
			//showUpdatedMap = false;
		}

		if(showUpdatedMap or updateCamera)
		{
			updateCamera = false;
			showUpdatedMap = false;
			cv::Vec3d t(poseNow.translation.x,poseNow.translation.y,poseNow.translation.z);
			cv::Matx33d R(poseNow.rotation);

			window->setWidgetPose("camPose",cv::Affine3d(R,t));

			cv::Affine3d viewerPose;

			switch (CAM_MODE){
			case CAM_OVERHEAD:
			{
				viewerPose.rotation(cv::Vec3d(-M_PI_2,0,0));
				viewerPose.translation(cv::Vec3d(t(0),t(1)-10,t(2)));
				window->setViewerPose(viewerPose);
				break;
			}
			case CAM_CURRENT_POSE:
			{
				viewerPose.rotation(R);
				viewerPose.translation(cv::Vec3d(t(0),t(1),t(2)));
				window->setViewerPose(viewerPose);
				break;
			}
			case CAM_FREE : break;
			default: break;
			}
		}
		window->spinOnce(1,true);  // Update Display

	}
	window->removeAllWidgets();
	window->close();
	windowImage->close();

	return true;
}

void MapVisualizer::showCloud(const std::vector<ORB_Types::Point3d>& points, const char* name, const cv::viz::Color& c)
{
	std::vector<cv::Vec3d> pointCloud;
	for(auto &p: points)	pointCloud.push_back(cv::Vec3d(p.x,p.y,p.z));
	if(!pointCloud.empty())
	{
		window->showWidget(name,viz::WCloud(pointCloud,c));
		window->setRenderingProperty(name,viz::POINT_SIZE,pointSize);
	}

}

MapVisualizer::MapVisualizer():
				window(new viz::Viz3d("Map Viewer")),
				windowImage(new viz::Viz3d("Image")),
				lcmInstance(new lcm::LCM()),
				mapUpdated(false),
				imageUpdated(false),
				poseUpdated(false),
				quit(false),
				updateCamera(false),
				pointSize(4),
				CAM_MODE(CAM_OVERHEAD)
{
	window->showWidget("Reference",viz::WCoordinateSystem(1));
	window->setBackgroundColor(cv::viz::Color(127),cv::viz::Color(200));
	window->setWindowSize(cv::Size2i(640,480));
	window->spinOnce(1,true);  // Update Display
	window->registerKeyboardCallback(&KeyboardCallback,this);
	window->wasStopped();


	std::vector<cv::Vec3d> p; p.push_back(cv::Vec3d(0,0,0));
	window->showWidget("camPose",viz::WCloud(p,cv::viz::Color::white()));
	window->setRenderingProperty("camPose",viz::POINT_SIZE,20);
}

MapVisualizer::~MapVisualizer()
{
	delete lcmInstance;
}
void MapVisualizer::listenTo(const std::string& MapChannel, const std::string& ImageChannel, const std::string& currentPoseChannel)
{
	lcmInstance->subscribe(MapChannel,   &MapVisualizer::MapReceiver,   this);
	lcmInstance->subscribe(ImageChannel, &MapVisualizer::ImageReceiver, this);
	lcmInstance->subscribe(currentPoseChannel, &MapVisualizer::PoseReceiver, this);

	std::cerr<<"Installed listener(s) "<<std::endl;
	while(lcmInstance->handle() == 0);
}


void KeyboardCallback(const cv::viz::KeyboardEvent& e, void* member)
{
	if(e.action == 0)
		static_cast<MapVisualizer*>(member)->keyboardCallback(e);
}

}



