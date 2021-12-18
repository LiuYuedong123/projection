#include "Point.h"

Point Point::operator+(const Point& p)
{
	float tmpx = this->_x + p.X();
	float tmpy = this->_y + p.Y();
	Point tmp(tmpx,tmpy);

	return tmp;
}
Point Point::operator-(const Point& p)
{
	float tmpx = this->_x - p.X();
	float tmpy = this->_y - p.Y();
	Point tmp(tmpx, tmpy);
	return tmp;
}
Point Point::operator*(const Point& p)
{
	float tmpx = this->_x * p.X();
	float tmpy = this->_y * p.Y();
	Point tmp(tmpx, tmpy);
	return tmp;
}
Point Point::operator*(float coff)
{

	float tmpx = this->_x * coff;
	float tmpy = this->_y * coff;
	Point tmp(tmpx, tmpy);
	return tmp;
}
//Point operator + (const Point& p1, const Point& p2)
//{
//	float tmpX = p1.X() + p2.X();
//	float tmpY = p1.Y() + p2.Y();
//	Point tmp(tmpX,tmpY);
//	return tmp;
//}

Point Point::PolarToCaresian(float theta, float dis)
{
	Point CarPoint(dis * cos(theta), dis * sin(theta));
	return CarPoint;
	
}

float Point::Distance(const Point& p1, const Point& p2)
{

	return sqrt((p1.X() - p2.X()) * (p1.X() - p2.X()) + (p1.Y() - p2.Y()) * (p1.Y() - p2.Y()));

}

float Point::DistanceSuqare(const Point& p1, const Point& p2)
{
	return ((p1.X() - p2.X()) * (p1.X() - p2.X()) + (p1.Y() - p2.Y()) * (p1.Y() - p2.Y()));
}

Point Point::MidPoint(const Point& p1, const Point& p2)
{
	float midX = 0.5 * (p1.X() + p2.X());
	float midY = 0.5 * (p1.Y() + p2.Y());
	Point mid(midX, midY);
	return mid;
}
