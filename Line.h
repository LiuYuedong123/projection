#pragma once
#include"Point.h"
#include<iostream>
#define PI 3.141592653
using namespace std;
class Line
{
public:
	Line(){}
	Line(const Point& p1, const Point& p2);
	Line(float a, float b, float c);
	Line(float theta, float c );
	inline float GetAngle();
	float A() const { return _a; }
	float B() const { return _b; }
	float C() const { return _c; }
	float Theta() const { return _theta; }
	static Point PointOfIntersection(const Line& l1, const Line& l2);// ������ֱ�ߵĽ���
	static float DistanceOfLineAndPoint(const Point& p, const Line& l);
private:
	//  ֱ��һ��ʽ ��ax+by+c=0
	//Point _p1;
	//Point _p2;
	float _theta; //  ��X��н�
	float _a;
	float _b;
	float _c;
};

