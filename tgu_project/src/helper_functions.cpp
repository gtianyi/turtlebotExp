#include "helper_functions.h"

float normalize_angle(float angle){
	// reduces angles to be between [-pi, pi]
	float PI = 3.14159;

	float newAngle = fmod(angle, PI*2);
    if(newAngle > PI)
        newAngle -= PI*2;

    return newAngle;
}

pair<float, float> xy2grid(pair<float, float> point, Position origin, float grid_resolution){
    // Converts the local X,Y coordinates to the grid coordinate system
    float gridX = int(round((point.first-origin.x)/grid_resolution));
    float gridY = int(round((point.second-origin.y)/grid_resolution));
    return pair<float, float> (gridX,gridY);
}

pair<float, float> grid2xy(pair<float, float> gridPoint, Position origin, float grid_resolution){
    // Converts the grid coordinates to the local X,Y.
    float x = gridPoint.first * grid_resolution + origin.x;
    float y = gridPoint.second * grid_resolution + origin.y;
    return pair<float, float> (x,y);
}

unsigned xy2mapIndex(pair<float, float> point, int grid_sizeX, int grid_sizeY){
	// Generates an index for the given point in the map. 
	// (If map is too small for point then returns the maximum map size)
	if( point.first > grid_sizeX || point.second > grid_sizeY)
		return grid_sizeX * grid_sizeY -1;
	else
		return unsigned( point.second * grid_sizeY + point.first);
}

pair<float, float> mapIndex2xy(unsigned index, int grid_sizeX, int grid_sizeY){
    // Converts an index into an x,y point
    float x = index % int(grid_sizeX);
    float y = (index-x)/grid_sizeY;
    return pair<float, float> (x,y);
}
