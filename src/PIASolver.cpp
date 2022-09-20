#include <PIASolver.hpp>
#include <iostream>

PolarPoint PIASolver::solve(std::vector<float> vertices, float angleInc) {
    PolarPoint pia;
    pia.phi = 0;
    pia.r = 0;
    float distR = *std::max_element(vertices.begin(), vertices.end());
    float maxDist = 0;
    while (distR > accuracy) {
        unsigned int misses = 0;
        while (misses < max_misses) {
            PolarPoint relTmp;
            relTmp.phi = rand() % (int) (2 * M_PI * 100000) / 100000.0f;
		    relTmp.r = rand() % (int) (distR * 100000) / 100000.0f;
            PolarPoint tmp = globalFromLocal(pia, relTmp, distR);
		    if (isPointInPolygon(tmp, vertices, angleInc)) {
		    	float dist = getDistanceOfNearestVertice(vertices, angleInc, tmp);
		    	if (dist > maxDist) {
		    		maxDist = dist;
		    		pia = tmp;
		    		misses = 0;
		    	} else {
		    		misses++;
		    	}
		    }
        }
        distR /= 1.4f;
    }
    return pia;
}

PolarPoint PIASolver::globalFromLocal(PolarPoint globalFrameCenter, PolarPoint pointInFrame, float max) {
    PolarPoint absFrameCenter;
    float r1 = globalFrameCenter.r;
    float r2 = pointInFrame.r;
    float f1 = globalFrameCenter.phi;
    float f2 = pointInFrame.phi;
    float x = r1 * cosf(f1) + r2 * cosf(f2);
    float y = r1 * sinf(f1) + r2 * sinf(f2);
    absFrameCenter.r = sqrtf(x * x + y * y);
    absFrameCenter.phi = atan2f(y, x);
    if (absFrameCenter.phi < 0.0f) absFrameCenter.phi += 2 * M_PI;
    return absFrameCenter;
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
    int verInd2 = (verInd1 + 1) % (ranges.size() - 1);
    float phi2 = angleInc * verInd2;
    float y1 = ranges[verInd1] * sinf(phi1);
    float x1 = ranges[verInd1] * cosf(phi1);
    float y2 = ranges[verInd2] * sinf(phi2);
    float x2 = ranges[verInd2] * cosf(phi2);
    float m = y2 - y1;
    float n = x2 - x1;
    float k = tan(point.phi);
    float x = (x1 * y2 - x2 * y1) / (y2 - y1 - k * (x2 - x1));
    float y = k * x;
    float R = sqrt(x * x + y * y);
    return point.r < R;
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
