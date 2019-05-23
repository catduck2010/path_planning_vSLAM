#pragma once

#include <opencv2/core/core.hpp>
#include <list>
#include <set>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>

const float kCost1 = 1.0; //直移一格消耗
const float kCost2 = 1.4; //斜移一格消耗

struct PtA
{
	cv::Point2i pt;
	int F, G, H;
	PtA* parent;
	PtA(cv::Point2i _pt) : pt(_pt), F(0), G(0), H(0), parent(NULL)
	{
	}
	PtA(int x, int y) :F(0), G(0), H(0), parent(NULL) {
		pt = *(new cv::Point2i(x, y));
	}
};

class AStar
{
public:
	bool isCanreach(const PtA* point, const PtA* target, bool isIgnoreCorner) const;
	void mazeInit(std::list<cv::Point2f> _2Dpoints);//初始化迷宫同时界定地图大小
	void addPoint(float x, float y);
	void setSmallestThreshold(float threshold);
	void initSize();
	
	//std::list<PtA*> getPath(PtA& startPoint, PtA& endPoint, bool isIgnoreCorner, float startThreshold);
	//std::vector<PtA*> getSurroundPoints(const PtA* point, bool isIgnoreCorner, float threshold);
	PtA* isInList(const std::list<PtA*>& list, const PtA* point) const;             //判断开启/关闭列表中是否包含某点
	PtA* isNear(const std::list<PtA*>& list, const PtA* point, const int threshold) const;
	PtA* findPath(PtA& startPoint, PtA& endPoint, bool isIgnoreCorner, int threshold);
	std::list<PtA*> getPath(PtA& startPoint, PtA& endPoint, bool isIgnoreCorner, int threshold);
	std::vector<float> getMazeSize();
	PtA* getLeastFpoint();                                                           //从开启列表中返回F值最小的节点
	//PtA* findPath(PtA& startPoint, PtA& endPoint, bool isIgnoreCorner, float startThreshold);


protected:
	std::list<cv::Point2f> maze;
	float maze_xmax, maze_xmin, maze_ymax, maze_ymin;
	std::list<PtA*> openList;
	std::list<PtA*> closeList;
	float gridStep = 5.0;
	int currPathLength;
	float smallestThreshold = 5.0;
	//bool* checkGrid(const PtA* currPorint, float grid);//判断某点是否可以用于下一步判断
	//bool* checkDirection(PtA& currPoint, PtA& target);
	//bool* checkDirection(PtA& currPoint, cv::Point2i& target);
	bool checkEnd(PtA* currPoint, PtA* endPoint, float grid) const;
	std::vector<PtA*> getSurroundPoints(const PtA* point, bool isIgnoreCorner) const;
	int calcG(PtA* temp_start, PtA* point);
	int calcH(PtA* point, PtA* end);
	int calcF(PtA* point);
	int getSurroundNumber(PtA* point, int threshold);

};