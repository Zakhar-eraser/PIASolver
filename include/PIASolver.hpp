#ifndef _INCLUDE_PIASOLVER_HPP_
#define _INCLUDE_PIASOLVER_HPP_

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

class PIASolver;

struct Point {
  float x, y; 
};

struct PolarPoint {
  float phi, r;
};

  
class PIASolverDestroyer {
  private:
    PIASolver* p_instance;
  public:
    ~PIASolverDestroyer();
    void initialize( PIASolver* p );
};

class PIASolver {
  private:
    static PIASolver* p_instance;
    static PIASolverDestroyer destroyer;
    const float accuracy = 0.1f;
    const unsigned int max_misses = 50;
    const unsigned int density = 20; // checking points count on a start circle perimeter
    const unsigned int radDel = 10; // delimeter of circle radius-vector

    bool isPointInPolygon(PolarPoint point, std::vector<float> &ranges, float angleInc);
    float getDistanceToNearestEdge(std::vector<float> &vertices, float angleInc, PolarPoint point);
    PolarPoint globalFromLocal(PolarPoint globalFrameCenter, PolarPoint pointInFrame);
  protected:
    PIASolver() { }
    PIASolver( const PIASolver& );
    PIASolver& operator=( PIASolver& );
   ~PIASolver() { }
    friend class PIASolverDestroyer;
  public:

    static PIASolver& getInstance();
    PolarPoint solveMonteCarlo(std::vector<float> vertices, float angleInc);
    PolarPoint solve(std::vector<float> vertices, float angleInc);
};
#endif  // _INCLUDE_PIASOLVER_HPP_
