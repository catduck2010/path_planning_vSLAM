#include <opencv2/core/core.hpp>
#include <list>
#include <set>
#include <iostream>
#include "AStar.h"
#include <vector>
#include <math.h>

class RouteMap
{
public:
    //RouteMap();
    void AddPoint(cv::Mat pt);
    
    //std::set<cv::Mat> GetRoute(cv::Point2i startPt);
    std::list<cv::Point2f> GetRoute(cv::Point3f startPt);
    void setCamCoodinate(const cv::Mat pos);
    void setTwc(const cv::Mat _twc);
    //void run();
protected:
    std::list<cv::Mat> points;
    std::list<cv::Point2f> _2Dpoints;
    bool converted=false;
    cv::Mat Ow,Twc;
    unsigned int scale;

//protected:
    std::list<cv::Point2f> getPossibleEnds(cv::Mat &_Ow, std::vector<float> &size, int step);
    cv::Point2f getPossibleEnd(cv::Mat &_Ow, std::vector<float> const &size);
    cv::Point2f getPossibleEnd(cv::Mat &_Ow, std::vector<float> const &size, int angle);
    cv::Point2f getFarthestPoint(cv::Mat &_Ow, std::list<cv::Point2f> points);
    void debug(std::string str);
    void Convert2D();
    void Convert2D(unsigned int scale);
    void pathRecover(std::list<cv::Point2i> path);
    //void Classify();
    //void DrawBorder();
    void CalcRoute();
    bool isConverted();
    void setScale(unsigned int _scale);
};