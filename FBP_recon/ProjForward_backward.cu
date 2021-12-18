#include"DataStruct.h"
#include"Line.h"
#include"Point.h"
#include <math.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#define PI 3.141592653
__global__ void meshLinePosition(Line* rowLine, float sizePixel, float center, float numPixel, float a, float b)
{  //  ����ͼ�����ر߽��������

	for (size_t i = 0; i < numPixel + 1; i++)
	{
		float c = i * sizePixel - center;
		Line tmp(a, b, c);
		rowLine[i] = tmp;
	}
}
__global__ void IntersectPointArray(const Line& Lsd, const Line* meshLine, vector<Point>& pointArray, const ImgParams& imgPar)
{	// ���������������ߵĽ��㣬
	float imgCtrX = imgPar.centerX;
	float imgCtrY = imgPar.centerY;
	float imgLenX = imgPar.sizeX * imgPar.pixelSizeX;
	float imgLenY = imgPar.sizeY * imgPar.pixelSizeY;

	// ��������ͼ���ÿ�еĽ�������ꣻ�ڱ߽�ʱ������ֻ�벿�������ཻ
	auto border = [](float point, float ctr, float len) {return (point <= len - ctr && point >= -ctr); };
	for (size_t i = 0; i < meshLine.size(); i++)
	{
		Point tmp = Line::PointOfIntersection(Lsd, meshLine[i]);
		size_t count = 0;
		if (border(tmp.X(), imgCtrX, imgLenX) && border(tmp.Y(), imgCtrY, imgLenY))
		{
			pointArray.push_back(tmp);
		}

	}
}
__global__ void InterPointMesh(const Line& lsd, Line* rowMesh, Line* colMesh, ImgParams* imgPar, Point* interPoint)
{   //  ������������񽻵㣬�������������������
	vector<Point> rowPoint;  //  ���еĽ���
	vector<Point> colPoint;  //  ���еĽ���
	//������������������ߵĽ���
	IntersectPointArray(lsd, rowMesh, rowPoint, imgPar);
	IntersectPointArray(lsd, colMesh, colPoint, imgPar);
	//  �����������鰴���� �������У��������ߵļнǣ�
	if (lsd.Theta() >= 0 && lsd.Theta() <= PI / 2) // �н�С��90
	{
		sortMergeArray(rowPoint, colPoint, interPoint);
	}
	else  //  �нǴ���90  
	{
		vector<Point> colPoint2;//  col �ĵ���  ���нǴ���90ʱ��X��Y�ɷ���
		for (size_t i = 0; i < colPoint.size(); i++)
		{
			colPoint2.push_back(colPoint[colPoint.size() - 1 - i]);
		}
		sortMergeArray(rowPoint, colPoint2, interPoint);
	}
}
__global__ void backwardProjector(ImgParams* imgPar, float* d_detLine, float* d_proj,float* d_rowLine,
	float* d_colLine)
{





}
extern void ProjBackwardLD(ImgParams* imgPar, DetParams* detPar, float* proj, float* img)
{
	float SOD = detPar->SOD;
	float SDD = detPar->SDD;
	float ctrDet = detPar->centerU; // ʵ��λ�ã�mm��
	float unitSize = detPar->detColSize;
	size_t detNum = detPar->detCol;
	float detLen = detNum * unitSize;
	float projNum = detPar->projNum;
	size_t imgNumX = imgPar->sizeX;
	size_t imgNumY = imgPar->sizeY;
	float pixelSizeX = imgPar->pixelSizeX;
	float pixelSizeY = imgPar->pixelSizeY;
	float imgCtrX = imgPar->centerX;//  ʵ��λ�ã�mm��
	float imgCtrY = imgPar->centerY;

	Point* detCtrPos=new Point[projNum];   //  ̽�����е�����
	float* angleVec=new float[projNum];    //  ͶӰ�Ƕ�
	Point* sourPos = new Point[projNum];     //  Դ����
	for (size_t i = 0; i < projNum; i++)
	{
		float theta = (i * 2 * PI) / projNum;
		angleVec[i] = theta;
		Point tmp((SDD - SOD) * cos(PI + theta), (SDD - SOD) * sin(PI + theta));
		detCtrPos[i] = tmp;
		Point tmp1(SOD * cos(theta), SOD * sin(theta));
		sourPos[i] = tmp1;
	}
	float* detPosRel = new float[detNum];//  ����̽�����ĵ�Ԫ����е������,һά
	for (size_t i = 0; i < detNum; i++)
	{
		float temp = (0.5 + i) * unitSize - ctrDet;
		detPosRel[i] = temp;
	}
	Line* rowLine = new Line[pixelSizeY + 1];//  ��������������
	Line* colLine = new Line[pixelSizeX + 1];
	meshLinePosition(rowLine, pixelSizeY, imgCtrY, imgNumY, 0, -1);
	meshLinePosition(colLine, pixelSizeX, imgCtrX, imgNumX, -1, 0);
	Line* detLine = new Line[projNum * detNum];
	for (size_t p = 0; p < projNum; p++)  //  Ԥ�Ȱ��õ������ݴ�������������һ��Ҫ������
	{
		for (size_t i = 0; i < detNum; i++)
		{
			Point Angle(sin(angleVec[p]), -cos(angleVec[p]));
			Point unitPos = detCtrPos[p] + Angle * detPosRel[i];
			Line temp(unitPos, sourPos[p]);
			detLine[i + p * detNum] = temp;
		}
	}
	delete[] detCtrPos;
	delete[] angleVec;
	delete[] sourPos;
	delete[] detPosRel;
	//  ��������ͶӰ



	delete[] rowLine;
	delete[] colLine;
	delete[] detLine;
}