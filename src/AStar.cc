#include "AStar.h"

int AStar::calcG(PtA* temp_start, PtA* point)
{
	int extraG = (abs(point->pt.x - temp_start->pt.x) + abs(point->pt.y - temp_start->pt.y)) == 1 ? kCost1 : kCost2;
	int parentG = (point->parent == NULL) ? 0 : point->parent->G;
	//如果是初始节点，则其父节点是空
	return parentG + extraG;
}
int AStar::calcH(PtA * point, PtA * end)
{
	int a = (int)sqrt((double)(end->pt.x) * (double)(point->pt.x) + (double)(end->pt.y) * (double)(point->pt.y)) * kCost1;
	int b = 3 * getSurroundNumber(point, 5);
	return a + b;
}
int AStar::calcF(PtA * point)
{
	return point->G + point->H;
}

int AStar::getSurroundNumber(PtA* point, int threshold)
{
	bool covered=false;
	//a[i] the i+1 -th quadrant
	int px = point->pt.x, py = point->pt.y, num=0;
	for (std::list<cv::Point2f>::iterator it = maze.begin(), ed = maze.end(); it != ed; it++)
	{
		if (px - threshold <= it->x && it->x <= px + threshold &&
			py - threshold <= it->y && it->y <= py + threshold)
		{
			num++;
		}
		if (!covered &&
			abs(px - it->x) < 1.0 && abs(py - it->y) < 1.0)
		{
			covered = true;
		}

	}
	if (covered) {
		num *= 10;
	}
	return num;
}

PtA* AStar::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto& point : openList)
			if (point->F < resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

PtA* AStar::isInList(const std::list<PtA*> & list, const PtA * point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
		if (p->pt.x == point->pt.x && p->pt.y == point->pt.y)
			return p;
	return NULL;
}

PtA* AStar::isNear(const std::list<PtA*>& list, const PtA* point, const int threshold) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
		if (abs(p->pt.x - point->pt.x) + abs(p->pt.y == point->pt.y) <= threshold)
			return p;
	return NULL;
}


PtA * AStar::findPath(PtA & startPoint, PtA & endPoint, bool isIgnoreCorner=false, int threshold=5)
{
	openList.push_back(new PtA(*(new cv::Point2f(startPoint.pt.x, startPoint.pt.y)))); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1,找到当前周围八个格中可以通过的格子
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto& target : surroundPoints)
		{
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG < target->G)
				{
					target->parent = curPoint;

					target->G = tempG;
					target->F = calcF(target);
				}
			}
			PtA* resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
			/*else {
				resPoint = isNear(openList, &endPoint, threshold);
				if (resPoint)
					return resPoint;
			}*/
		}
		}
	return NULL;

}


std::list<PtA*> AStar::getPath(PtA & startPoint, PtA & endPoint, bool isIgnoreCorner, int threshold)
{
	PtA* result = findPath(startPoint, endPoint, isIgnoreCorner,threshold);
	std::list<PtA*> path;
	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}

	// 清空临时开闭列表，防止重复执行GetPath导致结果异常
	openList.clear();
	closeList.clear();

	return path;
}

//下面两个需要在float域想办法
//std::vector<PtA*> AStar::getSurroundPoints(const PtA * point, bool isIgnoreCorner, float threshold = 20.0)
//{
//	std::vector<PtA*> surroundPoints;
//	bool find = false;
//	while (!find)
//	{
//		bool* quad = checkGrid(point, threshold);
//		if (quad[0])
//		{
//			surroundPoints.push_back(new PtA(
//				Round(point->pt.x + randomHalf2One() * threshold,1), 
//				Round(point->pt.y + randomHalf2One() * threshold,1)));
//			find = true;
//		}
//		if (quad[1])
//		{
//			surroundPoints.push_back(new PtA(
//				Round(point->pt.x + randomHalf2One() * threshold,1),
//				Round(point->pt.y - randomHalf2One() * threshold,1)));
//			find = true;
//		}
//		if (quad[2])
//		{
//			surroundPoints.push_back(new PtA(
//				Round(point->pt.x - randomHalf2One() * threshold, 1),
//				Round(point->pt.y - randomHalf2One() * threshold, 1)));
//			find = true;
//		}
//		if (quad[3])
//		{
//			surroundPoints.push_back(new PtA(
//				Round(point->pt.x - randomHalf2One() * threshold, 1),
//				Round(point->pt.y + randomHalf2One() * threshold, 1)));
//			find = true;
//		}
//		if (!find && threshold < smallestThreshold)
//		{
//			threshold -= 0.5f;
//		}
//	}
//
//	// for (int x = point->pt.x - threshold; x <= point->pt.x + threshold; x++)
//	// 	for (int y = point->pt.y - threshold; y <= point->pt.y + threshold; y++)
//	// 		if (isCanreach(point, new PtA(x, y), isIgnoreCorner))
//	// 			surroundPoints.push_back(new PtA(x, y));
//
//	return surroundPoints;
//}

 bool AStar::isCanreach(const PtA *point, const PtA *target, bool isIgnoreCorner) const
 {
 	if (target->pt.x < maze_xmin || target->pt.x > maze_xmax ||
 		target->pt.y < maze_ymin || target->pt.y > maze_ymax ||
 		target->pt.x == point->pt.x && target->pt.y == point->pt.y ||
 		isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
 		return false;
 	//下面的试一下怎么改
 	else
 	{
 		if (abs(point->pt.x - target->pt.x) + abs(point->pt.y - target->pt.y) == 1) //非斜角可以
 			return true;
 		else
 		{
 			////斜对角要判断是否绊住
 			//if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
 			//	return true;
 			//else
 				return isIgnoreCorner;
 		}
 	}
 }

void AStar::mazeInit(std::list<cv::Point2f> _2Dpoints)
{
	//initialize maze and get the size of it
	float cx, cy;
	if (maze.size()<5) {
		return;
	}
	for (std::list<cv::Point2f>::iterator it = _2Dpoints.begin(), ed = _2Dpoints.end(); it != ed; it++)
	{
		maze.push_back(*new cv::Point2f(it->x, it->y));
		if (it == _2Dpoints.begin())
		{
			maze_xmax = it->x;
			maze_xmin = it->x;
			maze_ymax = it->y;
			maze_ymin = it->y;
			continue;
		}
		cx = it->x;
		cy = it->y;
		if (cx > maze_xmax)
			maze_xmax = cx;
		if (cx < maze_xmin)
			maze_xmin = cx;
		if (cy > maze_ymax)
			maze_ymax = cy;
		if (cy < maze_ymin)
			maze_ymin = cy;

		maze_xmax+=0.5;
		maze_xmin-=0.5;
		maze_ymax+=0.5;
		maze_ymin-=0.5;
	}
}

//bool* AStar::checkGrid(const PtA * currPorint, float grid)
//{
//	bool a[4] = { true, true, true, true }; //true=no points here
//	//a[i] the i+1 -th quadrant
//	float cx, cy;
//	float px = currPorint->pt.x, py = currPorint->pt.y;
//	for (std::list<cv::Point2i>::iterator it = maze.begin(), ed = maze.end(); it != ed; it++)
//	{
//		cx = it->x;
//		cy = it->y;
//		//+grid,+grid a[0]
//		if (a[0])
//		{
//			if (px <= cx && cx <= px + grid && py <= cy && cy <= py + grid)
//			{
//				a[0] = false;
//			}
//		}
//		//+grid,-grid a[1]
//		if (a[1])
//		{
//			if (px <= cx && cx <= px + grid && py - grid <= cy && cy <= py)
//			{
//				a[1] = false;
//			}
//		}
//		//-grid, -grid a[2]
//		if (a[2])
//		{
//			if (px - grid <= cx && cx <= px && py - grid <= cy && cy <= py)
//			{
//				a[2] = false;
//			}
//		}
//		//-grid, +grid a[3]
//		if (a[3])
//		{
//			if (px - grid <= cx && cx <= px && py <= cy && cy <= py + grid)
//			{
//				a[3] = false;
//			}
//		}
//		if (!(a[0] && a[1] && a[2] && a[3]))
//		{
//			break;
//		}
//	}
//	return a;
//}

//bool* AStar::checkDirection(PtA & currPoint, PtA & target)
//{
//	bool a[4] = { false, false, false, false }; //true=no points here
//	float deltax = target.pt.x - currPoint.pt.x, deltay = target.pt.y - currPoint.pt.y;
//	//a[i] the i+1 -th quadrant
//	//+grid,+grid a[0]
//	if (deltax == 0 && deltay == 0)
//	{
//		return a;
//	}
//	else
//	{
//		if (deltax >= 0 && deltay >= 0)
//		{
//			a[0] = true;
//		}
//		//+grid,-grid a[1]
//		if (deltax >= 0 && deltay <= 0)
//		{
//			a[1] = true;
//		}
//		//-grid, -grid a[2]
//		if (deltax <= 0 && deltay <= 0)
//		{
//			a[2] = true;
//		}
//		//-grid, +grid a[3]
//		if (deltax <= 0 && deltay >= 0)
//		{
//			a[3] = true;
//		}
//	}
//	return a;
//}
//
//bool* AStar::checkDirection(PtA& currPoint, cv::Point2f& target)
//{
//	bool a[4] = { false, false, false, false }; //true=no points here
//	float deltax = target.x - currPoint.pt.x, deltay = target.y - currPoint.pt.y;
//	//a[i] the i+1 -th quadrant
//	//+grid,+grid a[0]
//	if (deltax == 0 && deltay == 0)
//	{
//		return a;
//	}
//	else
//	{
//		if (deltax >= 0 && deltay >= 0)
//		{
//			a[0] = true;
//		}
//		//+grid,-grid a[1]
//		if (deltax >= 0 && deltay <= 0)
//		{
//			a[1] = true;
//		}
//		//-grid, -grid a[2]
//		if (deltax <= 0 && deltay <= 0)
//		{
//			a[2] = true;
//		}
//		//-grid, +grid a[3]
//		if (deltax <= 0 && deltay >= 0)
//		{
//			a[3] = true;
//		}
//	}
//	return a;
//}

void AStar::addPoint(float x, float y) {
	maze.push_back(*new cv::Point2f(x, y));
}

void AStar::initSize() {
	float cx, cy;
	if (maze.size() == 0) {
		return;
	}
	for (std::list<cv::Point2f>::iterator it = maze.begin(), ited = maze.end(); it != ited; it++)
	{
		if (it == maze.begin())
		{
			maze_xmax = it->x;
			maze_xmin = it->x;
			maze_ymax = it->y;
			maze_ymin = it->y;
			continue;
		}
		cx = it->x;
		cy = it->y;
		if (cx > maze_xmax)
			maze_xmax = cx;
		if (cx < maze_xmin)
			maze_xmin = cx;
		if (cy > maze_ymax)
			maze_ymax = cy;
		if (cy < maze_ymin)
			maze_ymin = cy;
	}
}

void AStar::setSmallestThreshold(float threshold) {
	smallestThreshold = threshold;
}

bool AStar::checkEnd(PtA * currPoint, PtA * endPoint, float grid) const
{
	if (!currPoint) {
		return false;
	}
	//bool a[4] = { false, false, false, false }; //true=no points here
	//a[i] the i+1 -th quadrant
	float deltax = endPoint->pt.x - currPoint->pt.x, deltay = endPoint->pt.y - currPoint->pt.y;
	//a[i] the i+1 -th quadrant
	//+grid,+grid a[0]
	if (deltax == 0 && deltay == 0)
	{
		return true;
	}

	if (abs(deltax) <= grid && abs(deltay) <= grid)
	{
		return true;
	}
	return false;
}


std::vector<PtA*> AStar::getSurroundPoints(const PtA* point, bool isIgnoreCorner) const
{
	std::vector<PtA*> surroundPoints;

	for (int x = point->pt.x - 1; x <= point->pt.x + 1; x++)
		for (int y = point->pt.y - 1; y <= point->pt.y + 1; y++)
			if (isCanreach(point, new PtA(x, y), isIgnoreCorner))
				surroundPoints.push_back(new PtA(x, y));

	return surroundPoints;
}

std::vector<float> AStar::getMazeSize(){
	std::vector<float> size={maze_xmax,maze_xmin,maze_ymax,maze_ymin};
	return size;
}