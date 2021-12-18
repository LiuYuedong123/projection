#pragma once
#include<math.h>
using namespace std;
class Point
{
public:
	Point() {}
	Point(float X,float Y):_x(X),_y(Y){}
	float X() const { return _x; }
	float Y() const { return _y; }
	Point operator + (const Point& p);
	Point operator - (const Point& p);
	Point operator*(const Point& p);
	Point operator*(float coff);
	Point& operator=(const Point& p) { this->_x = p._x; this->_y = p._y; return *this; }
	void setPointPosition(float x, float y) { _x = x; _y = y; }
	static Point PolarToCaresian(float theta, float dis);
	static float Distance(const Point& p1, const Point& p2);
	static float DistanceSuqare(const Point& p1, const Point& p2); // æ‡¿Î∆Ω∑Ω
	static Point MidPoint(const Point& p1, const Point& p2);
private:
	float _x;
	float _y;

};
