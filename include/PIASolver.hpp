#ifndef _INCLUDE_PIASOLVER_HPP_
#define _INCLUDE_PIASOLVER_HPP_

#include <vector>
#include <limits>

class PIASolver;

struct Point {
    float x, y; 
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
    const unsigned int max_misses = 15;
  protected:
    PIASolver() { }
    PIASolver( const PIASolver& );
    PIASolver& operator=( PIASolver& );
   ~PIASolver() { }
    friend class PIASolverDestroyer;
  public:

    static PIASolver& getInstance();
    Point solve(std::vector<float> &vertices, float angleInc);
};
#endif  // _INCLUDE_PIASOLVER_HPP_
