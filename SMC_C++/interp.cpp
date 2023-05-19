#include "interp.h"
#include<algorithm>

namespace longitude_control
{

Interp::Interp()
{
  // MAP for throttle
  // rows for driving force of -400:200:4000 N
  // columns for speed of 0:2:40 km/h
  float64 temp1[23][21]={
{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	2 },
{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	2,	2,	3,	5,	8 },
{1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	3,	3,  3,	4,	4,  5,	6,	8,	10,	12 },
{2,	2,	2,	2,	3,	3,	3,	3,	4,	4,	4,	4,	4,  5,	5,	7,	9,	10,	12,	12,	13 },
{3,	3,	3,	3,	4,	4,	4,	4,	4,	5,	5,	5,	6,	7,	8,	10,	12,	12,	13,	14,	15 },
{3,	3,	4,	4,	5,	5,	5,	5,	6,	6,	7,	8,	9,	10,	12,	12,	13,	13,	14,	15,	16 },
{4,	4,	4,	5,	5,	5,	6,	6,  7,	8,	9,	10,	11,	12,	12,	13,	14,	14,	15,	16,	17 },
{5,	5,	5,	5,	6,	6,	7,	7,	9,	9,	11,	11,	12,	13,	13,	14,	15,	16,	17,	18,	19 },
{6,	6,	7,	7,	8,	8,	9,	9,	11,	11,	12,	12,	13,	14,	15,	16,	17,	17,	18,	19,	20 },
{7,	7,	7,	8,	8,	9,	11,	11,	12,	12,	13,	14,	14,	15,	16,	17,	18,	19,	20,	21,	22 },
{8,	8,	8,	9,	11,	11,	13,	13,	14,	14,	14,	15,	16,	17,	17,	18,	19,	21,	21,	22,	23 },
{9,	9,	11,	11,	12,	12,	15,	15,	15,	15,	15,	16,	17,	18,	19,	20,	21,	22,	23,	23,	24 },
{11,11,	12,	12,	13,	15,	17,	17,	17,	17,	17,	18,	18,	20,	21,	22,	22,	23,	24,	25,	25 },
{12,12,	13,	13,	14,	16,	19,	19,	19,	19,	19,	19,	20,	21,	22,	23,	24,	24,	25,	26,	26 },
{12,13,	13,	14,	15,	17,	20,	20,	20,	20,	20,	21,	22,	23,	23,	24,	25,	25,	26,	27,	27 },
{13,14,	14,	15,	16,	18,	21,	21,	22,	22,	23,	23,	23,	24,	24,	25,	26,	27,	27,	28,	29 },
{14,15,	16,	17,	17,	19,	22,	22,	22,	22,	23,	23,	24,	25,	26,	26,	27,	28,	28,	29,	30 },
{15,16,	17,	18,	18,	20,	23,	23,	23,	23,	24,	25,	25,	26,	27,	27,	28,	29,	29,	30,	31 },
{16,17,	18,	19,	19,	21,	24,	24,	24,	24,	25,	26,	26,	27,	28,	28,	29,	30,	30,	31,	32 },
{18,18,	19,	20,	21,	23,	25,	25,	25,	25,	26,	27,	27,	28,	29, 30,	30,	31,	32,	32,	33 },
{19,19,	20,	22,	22,	24,	26,	25,	26,	26,	27,	28,	29,	29,	30, 31,	31,	32,	33,	33,	34 },
{20,21,	22,	23,	24,	25,	27,	27,	28,	28,	29,	29,	30,	31,	30,	32,	33,	32,	34,	35,	34 },
{22,23,	23,	23,	24,	26,	28,	29,	30,	30,	31,	31,	31,	32,	32,	32,	33,	34,	34,	35,	36 }
};
  // MAP for brake
  // braking force of 0:500:14000 N
  //float64 temp2[29] = { 15, 20.5, 20.6, 20.8, 21, 23, 23.5, 24, 24.5, 25, 25.4, 25.8, 26.2, 26.6, 27,
	//      27.4, 27.8, 28.2, 28.5, 28.8, 29.2, 29.6, 30, 30.3, 30.6, 30.9, 31.3, 31.7, 32.1 };
  float64 temp2[29] = { 0, 17.5, 20.6, 20.8, 21, 22.3, 23, 24, 25, 26, 27, 28, 29, 30, 31,
	      32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 };
  for (int32 i = 0; i < 23; i++)
  {   
	  std::vector<float64> tempvector;
	  for (int32 j = 0; j < 21; j++)
	  {
		  tempvector.push_back(temp1[i][j]);
	  }
	  thr_map.push_back(tempvector);
	  Ft_map_thr.push_back(-400 + 200 * i);
  }


  for (int32 i = 0; i < 29; i++)
  {  
	 
	 // tempvector.push_back(temp2[i]);
	  Fb_map_brk.push_back(500 * i);
	  brk_map.push_back(temp2[i]);
  }


  for (int32 i = 0; i < 21; i++)
  {
	  v_map.push_back(2 * i);
  }
}

Interp::~Interp()
{

}

void Interp::ClassInit()
{

}


float64 Interp::Interp2Thr(float64 v, float64 Ft)
{
	int32 index_column;
	if (int32(v) <= v_map.front())
	{
        index_column = 0;
	}
	else if (int32(v) <= v_map.back())
	{
        index_column = std::find(v_map.begin(), v_map.end(), int32(v / 2) * 2) - v_map.begin();
	}
	else
	{
        index_column = v_map.end() - v_map.begin() - 1;
	}
	//ROS_INFO("v = %d, max_v = %d, index_column:%d",int32(v),v_map.back(),index_column);
	int32 index_row;
	if (int32(Ft) <= Ft_map_thr.front())
	{
		index_row = 0;
	}
	else if (int32(Ft) <= Ft_map_thr.back())
	{
		index_row = std::find(Ft_map_thr.begin(), Ft_map_thr.end(), int32(Ft / 200) * 200) - Ft_map_thr.begin();
	}
	else
	{
		index_row = Ft_map_thr.end() - Ft_map_thr.begin() - 1;
	}
	//ROS_INFO("index_row:%d",index_row);
	float64 proportion = 0.5*((v/2 - int32(v/2) ) + (Ft/200 - int32(Ft/200) ));
    if (index_row == 22 )
	{
	    index_row = 21;
	}
	if (index_column == 20 )
	{
	    index_column = 19;
	}
	float64 thr = thr_map[index_row][index_column] + (thr_map[index_row + 1][index_column + 1] - thr_map[index_row][index_column]) *  proportion;
	std::cout << "\033[32m[drive_row : \033[0m" << index_row << std::endl;
	std::cout << "\033[32m[drive_column : \033[0m" << index_column << std::endl;
	std::cout << "\033[32m[drive_thr : \033[0m" << thr << std::endl;
	return thr;
}

float64 Interp::Interp2Brk(float64 Fb)
{
	int32 index_column;
	if (int32(Fb) <= Fb_map_brk.front())
	{
		index_column = 0;
	}
	else if (int32(Fb) <= Fb_map_brk.back())
	{
		index_column = std::find(Fb_map_brk.begin(), Fb_map_brk.end(), int32(Fb / 500) * 500) - Fb_map_brk.begin();
	}
	else
	{
		index_column = Fb_map_brk.end() - Fb_map_brk.begin() - 1;
	}
	float64 proportion = Fb/500 - int32(Fb / 500);
	if (index_column == 28 )
	{
		index_column = 27;
	}
	float64 brk = brk_map[index_column] + (brk_map[index_column + 1] - brk_map[index_column]) * proportion;
	std::cout << "\033[32m[brake_column : \033[0m" << index_column << std::endl;
	std::cout << "\033[32m[brake_brk : \033[0m" << brk << std::endl;
	return brk;
}

}
