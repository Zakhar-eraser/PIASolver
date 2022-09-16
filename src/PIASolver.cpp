#include <PIASolver.hpp>

Point PIASolver::solve(std::vector<float> &vertices, float angleInc) {
    Point pia;
    float error = std::numeric_limits<float>::infinity();
    while (error > accuracy) {
        unsigned int misses = 0;
        Point tmp;
        while (misses > max_misses) {
            tmp->x = bounds[0] + (rand() % (int) ((bounds[1] - bounds[0]) * 100000) / 100000.0f);
		    tmp->y = bounds[2] + (rand() % (int) ((bounds[3] - bounds[2]) * 100000) / 100000.0f);


		if (Geometry_is_point_in_polygon(polygon, tmp->x, tmp->y)) {
			tmp_distance = Geometry_distance_between_point_and_polygon(polygon, tmp);

			if (tmp_distance > max_distance) {
				max_distance = tmp_distance;
				pia->x = tmp->x;
				pia->y = tmp->y;
				count = 0;
			} else {
				count++;
			}
		}
        }
        
    }
}

PIASolver * PIASolver::p_instance = 0;
PIASolverDestroyer PIASolver::destroyer;
  
PIASolverDestroyer::~PIASolverDestroyer() {   
    delete p_instance; 
}

void PIASolverDestroyer::initialize( PIASolver* p ) {
    p_instance = p; 
}

PIASolver& PIASolver::getInstance() {
    if(!p_instance)     {
        p_instance = new PIASolver();
        destroyer.initialize( p_instance);     
    }
    return *p_instance;
}
