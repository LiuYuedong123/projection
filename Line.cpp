#include "Line.h"

Line::Line(const Point& p1, const Point& p2)
{

	_a = (p2.Y() - p1.Y());
	_b = (p2.X() - p1.X());
	_c = (p2.X() * p1.Y() - p2.Y() * p1.X());
	if (_a==0&&_b==0)
	{
		cout << "Line object is unavailable.";
		return ;
	}
	if (_a == 0) _theta=0;
	else if (_b == 0) _theta= PI / 2;
	else
	{
		float slope = -_a / _b;
		_theta = atan(slope)<0?atan(slope)+PI:atan(slope);//  [0,pi)
	}
}

Line::Line(float a, float b, float c):_a(a),_b(b),_c(c)
{

}

Line::Line(float theta, float c):_theta(theta),_c(c)
{
}


float Line::GetAngle()
{
	 return _theta;
}

Point Line::PointOfIntersection(const Line& l1, const Line& l2)
{
	float pY = (l1.C() * l2.A() - l2.C() * l1.A()) / (l1.A() * l2.B() - l1.B() * l2.A());
	float pX = (l1.C() * l2.B() - l2.C() * l1.B()) / (l1.B() * l2.A() - l1.A() * l2.B());
	Point PInter(pX, pY);
	return PInter;
}

float Line::DistanceOfLineAndPoint(const Point& p, const Line& l)
{
	return 0.0f;
}

