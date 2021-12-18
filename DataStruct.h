#pragma once
struct ImgParams
{
	size_t sizeX;
	size_t sizeY;
	float pixelSizeX;
	float pixelSizeY;
	float centerX;  // 实际长度(mm)
	float centerY;

};
struct DetParams
{
	size_t SDD;
	size_t SOD;
	size_t detCol;   //  探测单元数
	float detColSize; //  探测单元尺寸
	float centerU;  //  实际的长度 单位（mm）
	size_t projNum;
};