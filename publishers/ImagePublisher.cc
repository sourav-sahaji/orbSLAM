/*
 * ImagePublisher.cc
 *
 *  Created on: 22 May 2015
 *      Author: yasir
 */

#include "Utils.h"


int main(int argc, char** argv)
{
	int startFrom = 0;
	if(argc < 4)
	{
	    std::cerr<<"Usage: "<<argv[0]<<" fileList.txt channelName rate"<<std::endl;
	    return -1;
	}
	if(argc == 5) startFrom = atoi(argv[4]);

	std::cerr<<"File "<<argv[1]<<std::endl;
	std::cerr<<"Topic "<<argv[2]<<std::endl;
	std::cerr<<"Rate "<<atof(argv[3])<<std::endl;
	std::cerr<<"Starting from "<<startFrom<<std::endl;

	utils::image::RateLimitedImagePublisher p(atof(argv[3]),argv[2]);
	p.readFileAndPublish(argv[1],true,startFrom);
	return 0;
}

