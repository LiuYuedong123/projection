#include<iostream>
#include<vector>
#include"DataStruct.h"
#include"Line.h"

using namespace std;
struct position //保存像素点的二维索引
{
	size_t x;
	size_t y;
};
void sortMergeArray(const vector<Point>& a1, const vector<Point>& a2, vector<Point>& merge)
{// 将两个已经排好序的升序数组按照升序排成一个数组
	
	size_t i = 0, j = 0;
	while (i<a1.size()&&j<a2.size())
	{
		if (a1[i].Y()<a2[j].Y())
		{
			merge.push_back(a1[i]); ++i;
		}
		else {
			merge.push_back(a2[j]); ++j;
		}
	}
	if (i<a1.size())
	{
		for (size_t s = i; s < a1.size(); s++)
		{
			merge.push_back(a1[i++]);
		}
	}
	if (j<a2.size())
	{
		for (size_t s = j; s < a2.size(); s++)
		{
			merge.push_back(a2[j++]);
		}
	}
}
void IntersectPointArray(const Line& Lsd, const vector<Line>& meshLine, vector<Point>& pointArray, const ImgParams& imgPar)
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
		if (border(tmp.X(),imgCtrX,imgLenX)&& border(tmp.Y(), imgCtrY, imgLenY))
		{
			pointArray.push_back(tmp);
		}
		
	}
}
void InterPointMesh(const Line& lsd, const vector<Line>& rowMesh,const vector<Line>& colMesh,const ImgParams& imgPar,vector<Point>& interPoint )
{   //  获得射线与网格交点，按照纵坐标的升序排列
	vector<Point> rowPoint;  //  与行的交点
	vector<Point> colPoint;  //  与列的交点
	//计算射线与横纵网格线的交点
	IntersectPointArray(lsd, rowMesh, rowPoint, imgPar);
	IntersectPointArray(lsd, colMesh, colPoint, imgPar);
	//  两个交点数组按照沿 升序排列（根据射线的夹角）
	if (lsd.Theta() >= 0 && lsd.Theta() <=PI / 2) // 夹角小于90
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
void meshLinePosition(vector<Line>& rowLine, float sizePixel, float center,float numPixel,float a,float b)
{  //  生成图像像素边界的网格线
	
	for (size_t i = 0; i < numPixel+1; i++)
	{
		float c = i * sizePixel - center;
		Line tmp(a,b,c);
		rowLine.push_back(tmp);
	}
}
void pixelIndex2D(const vector<Point>& interPoint, vector<position>& pixelIndex, const ImgParams& imgPar)
{  // 计算射线穿过的像素的二维索引
	float imgCtrX = imgPar.centerX;
	float imgCtrY = imgPar.centerY;
	float psizeX = imgPar.pixelSizeX;
	float psizeY = imgPar.pixelSizeY; 
	position tmp;
	for (size_t i = 0; i < interPoint.size()-1; i++)
	{
		Point mid = Point::MidPoint(interPoint[i], interPoint[i + 1]);
		tmp.x = ((mid.X() + imgCtrX)/ psizeX);
		tmp.y = ((mid.Y() + imgCtrY)/ psizeY);
		pixelIndex.push_back(tmp);
	}
}
void pixelDistance(const vector<Point>& interPoint, vector<float>& dis)
{
	for (size_t i = 0; i < interPoint.size() - 1; i++)
	{
		float tmp = Point::Distance(interPoint[i], interPoint[i + 1]);
		dis.push_back(tmp);
	}
}
float pathIntegral(vector<float>& dis, const vector<position>& index2D,const vector<float>& image,const ImgParams&imgPar)
{	// 计算正投影的路径积分
	float integral = 0;
	size_t x,y;
	size_t imgSizeX = imgPar.sizeX;
	for (size_t i = 0; i < dis.size(); i++)
	{
		x = index2D[i].x;
		y = index2D[i].y;
		integral += dis[i] * image[x + imgSizeX * y];
	}
	return integral;
}
void pixelAccumulation(vector<float>& image,float proj, vector<float>& dis, const vector<position>& index2D, const vector<Point>& interPoint, const ImgParams& imgPar)
{	// 计算反投影像素值的累加
	float totalDis = Point::Distance(interPoint[0], interPoint[interPoint .size()-1]);
	size_t imgSizeX = imgPar.sizeX;
	size_t x, y;
	for (size_t i = 0; i < index2D.size(); i++)
	{
		x = index2D[i].x;
		y = index2D[i].y;
		image[x + imgSizeX * y]  = image[x + imgSizeX * y] + (proj*dis[i]) / totalDis;
	}
}
void ProjForward(const vector<float>& image, vector<float>& proj ,const ImgParams& imgPar,const DetParams& detPar) {//  正投影
	
	float SOD = detPar.SOD;
	float SDD = detPar.SDD;
	float ctrDet = detPar.centerU; // 实际位置（mm）
	float unitSize = detPar.detColSize;
	size_t detNum = detPar.detCol;
	float detLen = detNum * unitSize;
	float projNum = detPar.projNum;
	size_t imgNumX = imgPar.sizeX;
	size_t imgNumY = imgPar.sizeY;
	float pixelSizeX = imgPar.pixelSizeX;
	float pixelSizeY = imgPar.pixelSizeY;
	float imgCtrX = imgPar.centerX;//  实际位置（mm）
	float imgCtrY = imgPar.centerY;

	vector<Point> detCtrPos;   //  探测器中点坐标
	vector<float> angleVec;    //  投影角度
	vector<Point> sourPos;     //  源坐标
 	for (size_t i = 0; i < projNum; i++)
	{
		float theta = (i * 2*PI) / projNum;
		angleVec.push_back(theta);
		Point tmp((SDD - SOD) * cos(PI + theta), (SDD - SOD) * sin(PI + theta));
		detCtrPos.push_back(tmp);
		Point tmp1(SOD*cos(theta),SOD*sin(theta));
		sourPos.push_back(tmp1);
	}
	vector<float> detPosRel; //  线阵探测器的单元相对中点的坐标,一维
	for (size_t i = 0; i < detNum; i++)
	{
		float temp = (0.5 + i) * unitSize  - ctrDet;
		detPosRel.push_back(temp);			
	}
	vector<Line> rowLine;
	vector<Line> colLine;
	meshLinePosition(rowLine, pixelSizeY, imgCtrY, imgNumY, 0, -1);
	meshLinePosition(colLine, pixelSizeX, imgCtrX, imgNumX, -1, 0);
	vector <vector<Line>> detLine(projNum);//  所有投影的射线
	for (size_t p = 0; p < projNum; p++)  //  预先把用到的数据存起来，方便下一步要做并行
	{
		for (size_t i = 0; i < detNum; i++)
		{
			Point Angle(sin(angleVec[p]), -cos(angleVec[p]));
			Point unitPos = detCtrPos[p] +  Angle* detPosRel[i] ;
			Line temp(unitPos, sourPos[p]);
			detLine[p].push_back(temp);
		}
	}
	//  线驱动正投影
	vector<Point> pointArray;
	vector<position>  index2D;
	vector<float> disVec;
	for (size_t p = 0; p < projNum; p++)
	{
		for (size_t i = 0; i < detNum; i++)
		{
			// 计算每条射线与每行或每列像素的交点，//  pointArray 是按照纵坐标升序排列的射线与网格线的交点
			InterPointMesh(detLine[p][i], rowLine, colLine, imgPar, pointArray);
			if (pointArray.size()<=1)//  若没有交点，探测值为零
			{
				proj[i + p * detNum] = 0;
			}
			else {
				// 计算相交的像素块
				pixelIndex2D(pointArray, index2D, imgPar);
				// 计算与每个像素的距离（权重）
				pixelDistance(pointArray, disVec);
				// 正投影，计算路径积分 赋值给当前射线对应的探测单元
				proj[i + p * detNum] = pathIntegral(disVec, index2D, image, imgPar);

			}
			pointArray.clear();//  清空vector ，准备保存下一条射线的交点
			index2D.clear();
			disVec.clear();
		}
		
	}
}
void ProjBackward(vector<float>& image, const vector<float>& proj, const ImgParams& imgPar, const DetParams& detPar)//  反投影
{
	float SOD = detPar.SOD;
	float SDD = detPar.SDD;
	float ctrDet = detPar.centerU; // 实际位置（mm）
	float unitSize = detPar.detColSize;
	size_t detNum = detPar.detCol;
	float detLen = detNum * unitSize;
	float projNum = detPar.projNum;
	size_t imgNumX = imgPar.sizeX;
	size_t imgNumY = imgPar.sizeY;
	float pixelSizeX = imgPar.pixelSizeX;
	float pixelSizeY = imgPar.pixelSizeY;
	float imgCtrX = imgPar.centerX;//  实际位置（mm）
	float imgCtrY = imgPar.centerY;

	vector<Point> detCtrPos;   //  探测器中点坐标
	vector<float> angleVec;    //  投影角度
	vector<Point> sourPos;     //  源坐标
	for (size_t i = 0; i < projNum; i++)
	{
		float theta = (i * 2 * PI) / projNum;
		angleVec.push_back(theta);
		Point tmp((SDD - SOD) * cos(PI + theta), (SDD - SOD) * sin(PI + theta));
		detCtrPos.push_back(tmp);
		Point tmp1(SOD * cos(theta), SOD * sin(theta));
		sourPos.push_back(tmp1);
	}
	vector<float> detPosRel; //  线阵探测器的单元相对中点的坐标,一维
	for (size_t i = 0; i < detNum; i++)
	{
		float temp = (0.5 + i) * unitSize - ctrDet;
		detPosRel.push_back(temp);
	}
	vector<Line> rowLine;
	vector<Line> colLine;
	meshLinePosition(rowLine, pixelSizeY, imgCtrY, imgNumY, 0, -1);
	meshLinePosition(colLine, pixelSizeX, imgCtrX, imgNumX, -1, 0);
	vector <vector<Line>> detLine(projNum);//  所有投影的射线
	for (size_t p = 0; p < projNum; p++)  //  预先把用到的数据存起来，方便下一步要做并行
	{
		for (size_t i = 0; i < detNum; i++)
		{
			Point Angle(sin(angleVec[p]), -cos(angleVec[p]));
			Point unitPos = detCtrPos[p] + Angle * detPosRel[i];
			Line temp(unitPos, sourPos[p]);
			detLine[p].push_back(temp);
		}
	}
	//  线驱动反投影
	vector<Point> pointArray;
	vector<position>  index2D;
	vector<float> disVec;
	for (size_t p = 0; p < projNum; p++)
	{
		for (size_t i = 0; i < detNum; i++)
		{
			// 计算每条射线与每行或每列像素的交点，//  pointArray 是按照纵坐标升序排列的射线与网格线的交点
			InterPointMesh(detLine[p][i], rowLine, colLine, imgPar, pointArray);
			if (pointArray.size() > 1)//  若没有交点，像素增加值为0值为零
			{
				// 计算相交的像素块
				pixelIndex2D(pointArray, index2D, imgPar);
				// 计算与每个像素的距离（权重）
				pixelDistance(pointArray, disVec);
				//  反投影累加像素值
				pixelAccumulation(image,proj[p*detNum+i] ,disVec, index2D, pointArray, imgPar);
			}
			pointArray.clear();//  清空vector ，准备保存下一条射线的交点
			index2D.clear();
			disVec.clear();
		}

	}
}
int main() {
	//  
	//vector<Point> VecP1;
	//vector<Point> VecP2;
	//for (size_t i = 0; i < 8; i++)
	//{
	//	Point p(0, 2.4 * i + 1);
	//	Point q(0, -1.2*i + 10);
	//	VecP1.push_back(p);
	//	VecP2.push_back(q);
	//}
	//vector<Point> colPoint2;//  col 的倒序
	//for (size_t i = 0; i < VecP2.size(); i++)
	//{
	//	colPoint2.push_back(VecP2[VecP2.size() - i-1]);
	//}
	//vector<Point> me;
	//sortMergeArray(VecP1, colPoint2, me);
	//vector<double> vecDou;
	//for (size_t i = 0; i < 12; i++)
	//{
	//	vecDou.push_back(i);			
	//}
	//vecDou.clear();
	//vecDou.push_back(10);
	//Line lsd(4, -1, 1);
	ImgParams imgPar;
	imgPar.pixelSizeX = 0.1;
	imgPar.pixelSizeY = 0.1;
	imgPar.sizeX = 8;
	imgPar.sizeY = 8;
	imgPar.centerX = 0.4;
	imgPar.centerY = 0.4;
	DetParams detPar;
	detPar.projNum = 10;
	detPar.SDD = 2;
	detPar.SOD = 1;
	detPar.detCol = 10;
	detPar.detColSize = 0.2;
	detPar.centerU = 1;
	size_t imgLen = imgPar.sizeX * imgPar.sizeY;
	size_t projLen = detPar.projNum * detPar.detCol;
	vector<float> img;
	vector<float> proj(projLen);
	for (size_t i = 0; i < imgLen; i++)
	{
		img.push_back(1);
	}
	ProjForward(img, proj, imgPar, detPar);
	vector<float> reconImg;
	for (size_t i = 0; i < imgLen; i++)
	{
		reconImg.push_back(0);
	}
	vector<float> proj1;
	for (size_t i = 0; i < projLen; i++)
	{
		proj1.push_back(1);
	}
	ProjBackward(reconImg, proj1, imgPar, detPar);
	//vector<Line> meshY;
	//
	//vector<Point> pointArray;
	//meshLinePosition(meshY, imgPar.pixelSizeY, imgPar.centerY, imgPar.sizeY, 0, -1);
	//IntersectPointArray(lsd, meshY, pointArray, imgPar);
	//float a = 0.35;
	//double a1 = 0.35;//  连续的两个独立变量不是连续存储的
	//cout << &a << endl;
	//cout << &a1 << endl;
	//float aa[] = { 1,2,3,4 };
	//double bb[] = { 1,2,3,4 };
	//cout << aa << " "<<aa+1<<"  "<<aa+2<<"  "<<aa+3 <<endl;
	//cout << bb << " " <<bb+1<<"  "<<bb+2<<"  "<<bb+3 <<endl;
	//vector<float> detPosRel; //  线阵探测器的单元相对中点的坐标,一维
	//size_t  detNum = 100;
	//float ctrDet = 5;
	//float unitSize = 0.1;
	//for (size_t i = 0; i < detNum; i++)
	//{
	//	float temp = (0.5 + i) * unitSize - ctrDet;
	//	detPosRel.push_back(temp);
	//}
	//float mut=7;
	//Point P0(1, 4);
	//Point P1(3, 4.4);
	//Point P4 = P0 * mut;
	//Point P2 = P0 + P1;
	//Point P3 = P0 - P1;
	//Line a1(0, -1, 9);
	//Line b1(1, 3, -1);
	//Point p1=Line::PointOfIntersection(a1, b1);
	//Line c1(1, 2, 9);
	//Point p2 = Line::PointOfIntersection(b1, c1);
	//Line c2(1, 2, 4);
	//Point p3 = Line::PointOfIntersection(c1, c2);
	//vector<Line> rowLine;
	//meshLinePosition(rowLine, 0.2, 0.8, 8, 0, -1);
	//float the = PI / 4;
	//float bias = 0;
	//float a = atan(1);
	//float slope = 1;
	//float a1 = atan(slope) < 0 ? atan(slope) + PI : atan(slope);
	//vector<float> proj;
	//vector<float> img;

	return 0;
}