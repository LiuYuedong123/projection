#pragma once
struct ImgParams
{
	size_t sizeX;
	size_t sizeY;
	float pixelSizeX;
	float pixelSizeY;
	float centerX;  // ʵ�ʳ���(mm)
	float centerY;

};
struct DetParams
{
	size_t SDD;
	size_t SOD;
	size_t detCol;   //  ̽�ⵥԪ��
	float detColSize; //  ̽�ⵥԪ�ߴ�
	float centerU;  //  ʵ�ʵĳ��� ��λ��mm��
	size_t projNum;
};