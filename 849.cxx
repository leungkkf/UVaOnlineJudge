#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>

#define M_PI (3.14159)
#define TOL (1.0e-5)

class Point
{
  public:
    Point(const double x, const double y, const double theta, const double radius) : _x(x), _y(y), _theta(theta), _r(radius) { }
    Point(const double x, const double y) : _x(x), _y(y)
    {
      _r = sqrt(_x*_x + _y*_y); 
      _theta = atan2(_y, _x) * 180 / M_PI; 
    }
    Point(const Point& other) : _x(other._x), _y(other._y), _theta(other._theta), _r(other._r) { }
    ~Point() = default;

    double X() const { return _x; } 
    double Y() const { return _y; }
    double& X() { return _x; } 
    double& Y() { return _y; }
    double R() const
    {
      return _r; 
    }
    double Theta() const 
    {
      return _theta;
    }

    Point Normalised() const 
    {
      return Point(X() / R(), Y() / R(), Theta(), 1.0); 
    }

    Point operator-(const Point& other) const
    {
      const double dx = X() - other.X(); 
      const double dy = Y() - other.Y(); 
      return Point(dx, dy); 
    }

    double Dot(const Point& other) const 
    {
      return X()*other.X() + Y()*other.Y(); 
    }

    double Cross(const Point& other) const
    {
      return X()*other.Y() - other.X()*Y();
    }

  public:
    static Point FromQuestion(const double theta, const double radius)
    {
      const double a = Point::transformAngle(theta); 
      return Point(radius * cos(a * M_PI / 180.0), radius * sin(a * M_PI / 180.0), a, radius); 
    }
    static Point ToQuestion(const Point& p)
    {
      return Point(p.X(), p.Y(), Point::transformAngle(p.Theta()), p.R()); 
    }
    static bool LessAngle(const Point& lhs, const Point& rhs)
    {
      return lhs.Theta() < rhs.Theta(); 
    }

    static void CalculatePossibleVelocity(const Point& p1, const Point& p2, std::vector<Point>& velocityList, std::vector<double>& timeList)
    {
      bool isThroughOrigin = false;
      bool isPointToOrigin = false;
      const Point p1Norm = p1.Normalised();   
      const Point p2Norm = p2.Normalised();   
      const double dot = p2Norm.Dot(p1Norm); 
      const double cross = p2Norm.Cross(p1Norm);

      double diffTheta = 0.0; 
      if (p1.Theta() <= p2.Theta())
      {
        diffTheta = (360.0-p2.Theta()) + p1.Theta(); 
      }
      else if (p1.Theta() > p2.Theta())
      {
        diffTheta = p1.Theta() - p2.Theta();
      }
      const double diffTime = 2.0 * diffTheta / 360.0;
      //std::cout << "diffTime " << diffTime << std::endl;
      const Point diff = p2 - p1; 

      const double v1x = diff.X()/diffTime; 
      const double v1y = diff.Y()/diffTime; 
      const Point v1(v1x, v1y); 
      velocityList.push_back(v1); 
      //std::cout << "v1 " << v1x << "," << v1y << std::endl;
     
      if (std::fabs(std::fabs(dot) - 1.0) < TOL)
      {
        isPointToOrigin = true; 
        timeList.push_back(2.0); 
        if (std::fabs(dot - -1.0) < TOL)
        {
          timeList.push_back(2.0); 
          const double v2x = diff.X()/(diffTime + 2.0); 
          const double v2y = diff.Y()/(diffTime + 2.0); 
          const Point v2(v2x, v2y); 
          velocityList.push_back(v2); 
          isThroughOrigin = true; 
        }
      }
      else if (cross > 0.0)
      {
        double t1 = Solve(p2, velocityList[0], false, false);
        if (t1 > TOL * 10)
        { 
          timeList.push_back(t1);
        }
        else
        {
          timeList.clear();
          velocityList.clear();
        }
        //std::cout << "t1 " << t1 << std::endl;

        const double v2x = diff.X()/(diffTime + 2.0); 
        const double v2y = diff.Y()/(diffTime + 2.0); 
        const Point v2(v2x, v2y); 
        velocityList.push_back(v2); 
        double t2 = Solve(p2, velocityList[velocityList.size()-1], true, false); 
        timeList.push_back(t2);
      }
      else if (cross < 0.0)
      {
        double t = Solve(p2, velocityList[0], false, true); 
        timeList.push_back(t); 
      }
    }

  private:
    static double transformAngle(const double angle)
    { 
      // To our angle. 
      // 0   -> 90
      // 90  -> 0
      // 180 -> 270
      // 270 -> 180

      // To their angle. 
      // 90  -> 0
      // 0   -> 90
      // 270 -> 180
      // 180 -> 270
      double newAngle = (360.0 - angle) + 90.0; 
      if (newAngle >= 360.0)
      {
        newAngle -= 360; 
      }
      return newAngle; 
    }

    static double Solve(const Point& p, const Point& v, bool isSlowVelocity, bool isOppositeDirection)
    {
      double a = 0.0; 
      double b = 360.0;
      double startA = 0.0;  
      for (int i = 0; i < 2; ++i)
      {
        double offset = 0; 
        if (isSlowVelocity)
        {
          startA = 360.0; 
          a = 360.0; 
          b = 720.0; 
          offset = 360;
        }
        const double initalA = atan2(p.Y(), p.X());
        while (b - a > TOL)
        {
          const double mid = (a + b) / 2.0;
          const double t = mid / 180.0;  
          const double newPx = p.X() + v.X() * t; 
          const double newPy = p.Y() + v.Y() * t;
          double deltaA = (atan2(newPy, newPx) - initalA) * 180.0 / M_PI;
          //std::cout << "solve " << mid << " " << deltaA << " " << newPx << "," << newPy << " " << p.X() << "," << p.Y() << std::endl;

          if (isOppositeDirection)
          {
            deltaA = 360 - std::fabs(deltaA); 
          }

          if (std::fabs(deltaA) < mid - offset)
          {
            b = mid + (b-mid) / 2.0; 
          }
          else
          {
            a = mid - (mid-a) / 2.0; 
          }
        }
        if (std::fabs(a - startA) > TOL * 10 || isSlowVelocity)
        {
          return a / 180.0;
        }
        else
        {
          startA = 360.0; 
          a = 360.0;
          b = 720.0; 
        }  
      }

      //std::cout << "a " << a << ", startA " << startA << std::endl;
      if (std::fabs(a - startA) > TOL *10)
      {
        return a / 180;
      }
      return 0.0;  
    }

  private:
    double _x;
    double _y;
    double _theta; 
    double _r;
};


int main(int argc, char* argv[])
{
  double inputA1 = 0.0;
  double inputD1 = 0.0;
  double inputA2 = 0.0;
  double inputD2 = 0.0;

  std::cout << std::fixed;
  std::cout << std::setprecision(2);
  while(std::cin >> inputA1 >> inputD1 >> inputA2 >> inputD2) 
  {
    if (inputA1 == 0.0 && inputD1 == 0 && inputA2 == 0.0 && inputD2 == 0.0)
    {
      return 0; 
    }
    const Point p1 = Point::FromQuestion(inputA1, inputD1); 
    const Point p2 = Point::FromQuestion(inputA2, inputD2);
    std::vector<Point> possibleVeclocityList; 
    std::vector<double> possibleTimeList;
    std::vector<Point> results;  

    Point::CalculatePossibleVelocity(p1, p2, possibleVeclocityList, possibleTimeList); 
    
    for (unsigned int i = 0; i < possibleVeclocityList.size(); ++i)
    {
      const double px = p2.X() + possibleVeclocityList[i].X() * possibleTimeList[i]; 
      const double py = p2.Y() + possibleVeclocityList[i].Y() * possibleTimeList[i];
      
      const Point p(px, py);
      results.push_back(Point::ToQuestion(p)); 
      //std::cout << "p " << px << "," << py << std::endl; 
    }
    std::sort(results.begin(), results.end(), Point::LessAngle);

    for (unsigned int i = 0; i < results.size(); ++i)
    {
      std::cout << results[i].Theta() << " " << results[i].R();
      if (i != results.size() - 1)
      {
        std::cout << " ";
      }
      else
      {
        std::cout << std::endl;
      }
    }

  }
  return 0; 
} 