
#include <cmath>
#include <utility>

using namespace std;

typedef struct Position{
	float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
    float time = 0.0;
}Position;

float normalize_angle(float angle);
pair<float, float> xy2grid(pair<float, float> point, Position origin, float grid_resolution);
pair<float, float> grid2xy(pair<float, float> gridPoint, Position origin, float grid_resolution);
unsigned xy2mapIndex(pair<float, float> point, int grid_sizeX, int grid_sizeY);
pair<float, float> mapIndex2xy(unsigned index, int grid_sizeX, int grid_sizeY);

