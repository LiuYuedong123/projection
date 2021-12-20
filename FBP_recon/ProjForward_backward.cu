#include"DataStruct.h"
#include"Line.h"
#include"Point.h"
#include <math.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#define PI 3.141592653
struct position //�������ص�Ķ�ά����
{
	size_t x;
	size_t y;
};
void sortMergeArray(const vector<Point>& a1, const vector<Point>& a2, vector<Point>& merge)
{// �������Ѿ��ź�����������鰴�������ų�һ������

	size_t i = 0, j = 0;
	while (i < a1.size() && j < a2.size())
	{
		if (a1[i].Y() < a2[j].Y())
		{
			merge.push_back(a1[i]); ++i;
		}
		else {
			merge.push_back(a2[j]); ++j;
		}
	}
	if (i < a1.size())
	{
		for (size_t s = i; s < a1.size(); s++)
		{
			merge.push_back(a1[i++]);
		}
	}
	if (j < a2.size())
	{
		for (size_t s = j; s < a2.size(); s++)
		{
			merge.push_back(a2[j++]);
		}
	}
}
void pixelIndex2D(const vector<Point>& interPoint, vector<position>& pixelIndex, const ImgParams& imgPar)
{  // �������ߴ��������صĶ�ά����
	float imgCtrX = imgPar.centerX;
	float imgCtrY = imgPar.centerY;
	float psizeX = imgPar.pixelSizeX;
	float psizeY = imgPar.pixelSizeY;
	position tmp;
	for (size_t i = 0; i < interPoint.size() - 1; i++)
	{
		Point mid = Point::MidPoint(interPoint[i], interPoint[i + 1]);
		tmp.x = ((mid.X() + imgCtrX) / psizeX);
		tmp.y = ((mid.Y() + imgCtrY) / psizeY);
		pixelIndex.push_back(tmp);
	}
}
__global__ void meshLinePosition(Line* rowLine, float sizePixel, float center, float numPixel, float a, float b)
{  //  ����ͼ�����ر߽��������

	for (size_t i = 0; i < numPixel + 1; i++)
	{
		float c = i * sizePixel - center;
		Line tmp(a, b, c);
		rowLine[i] = tmp;
	}
}
__global__ void IntersectPointArray(Line Lsd, Line* meshLine,size_t mesh_size, Point* pointArray,ImgParams*imgPar)
{	// ���������������ߵĽ��㣬
	float imgCtrX = imgPar->centerX;
	float imgCtrY = imgPar->centerY;
	float imgLenX = imgPar->sizeX * imgPar->pixelSizeX;
	float imgLenY = imgPar->sizeY * imgPar->pixelSizeY;

	// ��������ͼ���ÿ�еĽ�������ꣻ�ڱ߽�ʱ������ֻ�벿�������ཻ
	auto border = [](float point, float ctr, float len) {return (point <= len - ctr && point >= -ctr); };
	for (size_t i = 0; i < mesh_size; i++)
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
__global__ void backwardProjector(ImgParams* imgPar,DetParams*detPar,float*d_image, Line* d_detLine, float* d_proj,
	Line* d_rowLine,Line* d_colLine)
{	
	size_t detNum = detPar->detCol;
	size_t projNum = detPar->projNum;

	size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
	size_t idy = blockIdx.y * blockDim.y + threadIdx.y;
	size_t tid = idx + idy * blockDim.x * gridDim.x;
	while (tid< detNum* projNum)  
	{
		Point* pointArray;
		// ����ÿ��������ÿ�л�ÿ�����صĽ���
		InterPointMesh(d_detLine[tid], d_rowLine, d_colLine, imgPar, pointArray);
		if (true)
		{
			// �����ཻ�����ؿ�
			pixelIndex2D(pointArray, index2D, imgPar);

		}

		tid += blockDim.x * gridDim.x * blockDim.y * gridDim.y;
	}



}
extern void ProjBackwardLD(ImgParams* imgPar, DetParams* detPar, float* proj, float* img)
{
	float SOD = detPar->SOD;
	float SDD = detPar->SDD;
	float ctrDet = detPar->centerU; // ʵ��λ�ã�mm��
	float unitSize = detPar->detColSize;
	size_t detNum = detPar->detCol;
	float detLen = detNum * unitSize;
	size_t projNum = detPar->projNum;
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
	}   /// ��һ���ӵ��豸������
	delete[] detCtrPos;
	delete[] angleVec;
	delete[] sourPos;
	delete[] detPosRel;
	//  ��������ͶӰ
	float* d_img;
	float* d_proj;
	Line* d_deline;
	Line* d_rowLine;
	Line* d_colLine;
	ImgParams* d_imgPar;
	DetParams* d_detPar;
	cudaMalloc((void**)&d_proj, sizeof(float) * projNum * detNum);
	cudaMalloc((void**)&d_deline, sizeof(Line) * projNum * detNum);// Ϊ���߿����Դ�projNum * detNum
	cudaMalloc((void**)&d_img, sizeof(float)* imgNumX* imgNumY);  // Ϊimg �����Դ�
	cudaMalloc((void**)&d_imgPar, sizeof(ImgParams));
	cudaMalloc((void**)&d_rowLine, sizeof(Line)*(pixelSizeY + 1));
	cudaMalloc((void**)&d_colLine, sizeof(Line)*(pixelSizeX + 1));
	cudaMalloc((void**)&d_detPar, sizeof(DetParams)); 

	cudaMemcpy(d_detPar, detPar, sizeof(ImgParams), cudaMemcpyHostToDevice);//  �������ݵ��Դ�
	cudaMemcpy(d_imgPar, imgPar, sizeof(DetParams), cudaMemcpyHostToDevice);
	cudaMemcpy(d_deline, detLine, sizeof(Line) * projNum * detNum, cudaMemcpyHostToDevice);
	cudaMemcpy(d_proj, proj, sizeof(float) * projNum * detNum, cudaMemcpyHostToDevice);
	cudaMemcpy(d_img, img, sizeof(float) * imgNumX * imgNumY, cudaMemcpyHostToDevice);
	cudaMemcpy(d_rowLine, rowLine, sizeof(Line) * (pixelSizeY + 1), cudaMemcpyHostToDevice);
	cudaMemcpy(d_colLine, colLine, sizeof(Line) * (pixelSizeX + 1), cudaMemcpyHostToDevice);

	delete[] rowLine;
	delete[] colLine;
	delete[] detLine;

	dim3 block(32, 32);
	dim3 grid(256, 32);

	backwardProjector << <grid, block >> > (d_imgPar, d_img, d_detPar,d_deline, d_proj, d_rowLine, d_colLine);
	cudaMemcpy(img, d_img, sizeof(float) * imgNumX * imgNumY, cudaMemcpyDeviceToHost);
	cudaFree(d_img);
	cudaFree(d_proj);
	cudaFree(d_deline);
	cudaFree(d_rowLine);
	cudaFree(d_colLine);
	cudaFree(d_imgPar);
}