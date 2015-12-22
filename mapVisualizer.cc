#include "Visualizer.h"




int main(int argc, char** argv)
{
	Visualizer::MapVisualizer m;
	std::thread listener(
			&Visualizer::MapVisualizer::listenTo,
			&m,
			"/map",
			"/trackedFrame",
			"/map/currentCamera"
	);
	m.UpdateLoop();

	return 0;
}


