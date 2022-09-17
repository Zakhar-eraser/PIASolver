#include <PIASolver.hpp>

PolarPoint PIASolver::solve(std::vector<float> vertices, float angleInc) {
    PolarPoint pia;
    pia.phi = 0;
    pia.r = 0;
    float error = std::numeric_limits<float>::infinity();
    float minPhi = 0;
    float maxPhi = 2 * M_PI;
    float minR = 0;
    float maxR = *std::max_element(vertices.begin(), vertices.end());
    float maxDist = 0;
    while (error > accuracy) {
        unsigned int misses = 0;
        PolarPoint tmp;
        while (misses < max_misses) {
            tmp.phi = pia.phi + (rand() % (int) ((maxPhi - minPhi) * 100000) / 100000.0f);
		    tmp.r = pia.r + (rand() % (int) ((maxR - minR) * 100000) / 100000.0f);

		    if (isPointInPolygon(tmp, vertices, angleInc)) {
		    	float dist = getDistanceOfNearestVertice(vertices, angleInc, tmp);

		    	if (dist > maxDist) {
		    		maxDist = dist;
		    		pia = tmp;
		    		misses = 0;
                    break;
		    	} else {
		    		misses++;
		    	}
		    }
        }
        error = maxR - minR;
        float deltaPhi = (maxPhi - minPhi) / (2 * sqrtf(2));
        float deltaR = (maxR - minR) / (2 * sqrtf(2));
        minPhi = tmp.phi - deltaPhi;
        maxPhi = tmp.phi + deltaPhi;
        minR = tmp.r - deltaR;
        if (minR < 0.0f) minR = 0;
        maxR = tmp.r + deltaR;
    }
    return pia;
}

float PIASolver::getDistanceOfNearestVertice(std::vector<float> &vertices, float angleInc, PolarPoint point) {
    float min = std::numeric_limits<float>::infinity();
    for (int i = 0; i < vertices.size(); i++) {
        float dist = sqrtf(point.r * point.r + vertices[i] * vertices[i] -
            2 * point.r * vertices[i] * cos(point.phi - angleInc * i));
        if (dist < min) {
            min = dist;
        }
    }
    return min;
}

bool PIASolver::isPointInPolygon(PolarPoint point, std::vector<float> &ranges, float angleInc) {
    int verInd1 = int (point.phi / angleInc);
    float phi1 = angleInc * verInd1;
    int verInd2 = verInd1 + 1;
    float phi2 = angleInc * verInd2;
    float k = ranges[verInd2] * sin(phi2) - ranges[verInd1] * sin(phi1) /
        ranges[verInd2] * cos(phi2) - ranges[verInd1] * cos(phi2);
    return point.r < (sin(phi1) + k * cos(phi1)) / (sin(point.phi) + k * cos(point.phi));
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
