#include <PIASolver.hpp>
#include <iostream>

PolarPoint PIASolver::solve(std::vector<float> vertices, float angleInc) {
    PolarPoint pia;
    pia.phi = 0;
    pia.r = 0;
    float distR = *std::max_element(vertices.begin(), vertices.end());
    float maxDist = 0;
    while (distR > accuracy) {
        for (unsigned int i = 1; i <= radDel; i++) {
            PolarPoint pnt;
            pnt.r = distR / i;
            const unsigned int angDel = i * density / radDel;
            const float inc = 2 * M_PI / angDel;
            pnt.phi = 0.f;
            for (unsigned int j = 0; j < angDel; j++, pnt.phi += inc) {
                PolarPoint tmp = globalFromLocal(pia, pnt);
                if (isPointInPolygon(tmp, vertices, angleInc)) {
                    float dist = getDistanceToNearestEdge(vertices, angleInc, tmp);
                    if (dist > maxDist) {
                        maxDist = dist;
                        pia = tmp;
                    }
                }
            }
        }
        distR /= 2.f;
    }
    return pia;
}

PolarPoint PIASolver::solveMonteCarlo(std::vector<float> vertices, float angleInc) {
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
		    relTmp.r = std::max(rand(), rand()) % (int) (distR * 100000) / 100000.0f;
            PolarPoint tmp = globalFromLocal(pia, relTmp);
		    if (isPointInPolygon(tmp, vertices, angleInc)) {
		    	float dist = getDistanceToNearestEdge(vertices, angleInc, tmp);
		    	if (dist > maxDist) {
		    		maxDist = dist;
		    		pia = tmp;
		    		misses = 0;
		    	} else {
		    		misses++;
		    	}
		    }
        }
        distR /= 1.2f;
    }
    return pia;
}

PolarPoint PIASolver::globalFromLocal(PolarPoint globalFrameCenter, PolarPoint pointInFrame) {
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

float PIASolver::getDistanceToNearestEdge(std::vector<float> &vertices, float angleInc, PolarPoint point) {
    float min = std::numeric_limits<float>::infinity();
    float phi = 0.f;
    for (int i = 0; i < vertices.size() - 1; i++, phi += angleInc) {
        float x0 = point.r * cosf(point.phi);
        float y0 = point.r * sinf(point.phi);
        float x1 = vertices[i] * cosf(phi);
        float y1 = vertices[i] * sinf(phi);
        int ni = (i + 1) % (vertices.size() - 1);
        float x2 = vertices[ni] * cosf(phi + angleInc);
        float y2 = vertices[ni] * sinf(phi + angleInc);
        float r;
        float deltaX = x2 - x1;
        float deltaY = y2 - y1;
        float y = (y0 * deltaY * deltaY + y1 * deltaX * deltaX + (x0 - x1) * deltaX * deltaY) /
            (deltaY * deltaY + deltaX * deltaX);
        float x = x1 + (y - y1) * deltaX / deltaY;
        if ((x > std::min(x1, x2)) && (x < std::max(x1, x2)) &&
            (y > std::min(y1, y2)) && (y < std::max(y1, y2))) {
            float xl = x - x0, yl = y - y0;
            r = xl * xl + yl* yl;
        } else {
            float xl1 = x1 - x0, xl2 = x2 - x0;
            float yl1 = y1 - y0, yl2 = y2 - y0;
            r = std::min(xl1 * xl1 + yl1 * yl1, xl2 * xl2 + yl2 * yl2);
        }
        if (min > r)
                min = r;
    }
    return sqrtf(min);
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
