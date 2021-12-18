#include"DataStruct.h"
#include"Line.h"
#include"Point.h"
#include <math.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#define PI 3.141592653
__global__ void meshLinePosition(Line* rowLine, float sizePixel, float center, float numPixel, float a, float b)
{  //  生成图像像素边界的网格线

	for (size_t i = 0; i < numPixel + 1; i++)
	{
		float c = i * sizePixel - center;
		Line tmp(a, b, c);
		rowLine[i] = tmp;
	}
}
__global__ void IntersectPointArray(const Line& Lsd, const Line* meshLine, vector<Point>& pointArray, const ImgParams& imgPar)
{	// 计算射线与网格线的交点，
	float imgCtrX = imgPar.centerX;
	float imgCtrY = imgPar.centerY;
	float imgLenX = imgPar.sizeX * imgPar.pixelSizeX;
	float imgLenY = imgPar.sizeY * imgPar.pixelSizeY;

	// 求射线与图像的每行的交点的坐标；在边界时，射线只与部分网格相交
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
{   //  获得射线与网格交点，按照纵坐标的升序排列
	vector<Point> rowPoint;  //  与行的交点
	vector<Point> colPoint;  //  与列的交点
	//计算射线与横纵网格线的交点
	IntersectPointArray(lsd, rowMesh, rowPoint, imgPar);
	IntersectPointArray(lsd, colMesh, colPoint, imgPar);
	//  两个交点数组按照沿 升序排列（根据射线的夹角）
	if (lsd.Theta() >= 0 && lsd.Theta() <= PI / 2) // 夹角小于90
	{
		sortMergeArray(rowPoint, colPoint, interPoint);
	}
	else  //  夹角大于90  
	{
		vector<Point> colPoint2;//  col 的倒序  当夹角大于90时，X与Y成反比
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
	float ctrDet = detPar->centerU; // 实际位置（mm）
	float unitSize = detPar->detColSize;
	size_t detNum = detPar->detCol;
	float detLen = detNum * unitSize;
	float projNum = detPar->projNum;
	size_t imgNumX = imgPar->sizeX;
	size_t imgNumY = imgPar->sizeY;
	float pixelSizeX = imgPar->pixelSizeX;
	float pixelSizeY = imgPar->pixelSizeY;
	float imgCtrX = imgPar->centerX;//  实际位置（mm）
	float imgCtrY = imgPar->centerY;

	Point* detCtrPos=new Point[projNum];   //  探测器中点坐标
	float* angleVec=new float[projNum];    //  投影角度
	Point* sourPos = new Point[projNum];     //  源坐标
	for (size_t i = 0; i < projNum; i++)
	{
		float theta = (i * 2 * PI) / projNum;
		angleVec[i] = theta;
		Point tmp((SDD - SOD) * cos(PI + theta), (SDD - SOD) * sin(PI + theta));
		detCtrPos[i] = tmp;
		Point tmp1(SOD * cos(theta), SOD * sin(theta));
		sourPos[i] = tmp1;
	}
	float* detPosRel = new float[detNum];//  线阵探测器的单元相对中点的坐标,一维
	for (size_t i = 0; i < detNum; i++)
	{
		float temp = (0.5 + i) * unitSize - ctrDet;
		detPosRel[i] = temp;
	}
	Line* rowLine = new Line[pixelSizeY + 1];//  像素网格线坐标
	Line* colLine = new Line[pixelSizeX + 1];
	meshLinePosition(rowLine, pixelSizeY, imgCtrY, imgNumY, 0, -1);
	meshLinePosition(colLine, pixelSizeX, imgCtrX, imgNumX, -1, 0);
	Line* detLine = new Line[projNum * detNum];
	for (size_t p = 0; p < projNum; p++)  //  预先把用到的数据存起来，方便下一步要做并行
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
	//  线驱动反投影



	delete[] rowLine;
	delete[] colLine;
	delete[] detLine;
}