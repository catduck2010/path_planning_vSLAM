#include "RouteMap.h"
#include <iostream>

using namespace std;
void RouteMap::AddPoint(cv::Mat pt){
    points.push_back(pt.clone());
}

void RouteMap::Convert2D(){
    //opengl uses right-hand model
    for(std::list<cv::Mat>::iterator itp=points.begin();itp!=points.end();itp++){
        cv::Mat coo=*itp;
        const double *data = itp->ptr<double>(0);
        cv::Point3d p(data[0], data[1], data[2]);
        //cv::Point2f pt=cv::Point2f(coo.at<float>(0),coo.at<float>(2));
        _2Dpoints.push_back(cv::Point2f(p.x,p.z));
    }
    points.clear();
    converted=true;
}

void RouteMap::Convert2D(unsigned int scale){
    if(scale<2){
        return;
    }
    //opengl uses right-hand model
    for(std::list<cv::Mat>::iterator itp=points.begin();itp!=points.end();itp++){
        cv::Mat coo=*itp;
        const double *data = itp->ptr<double>(0);
        cv::Point3d p(data[0], data[1], data[2]);
        //cv::Point2f pt=cv::Point2f(coo.at<float>(0),coo.at<float>(2));
        _2Dpoints.push_back(cv::Point2f(scale*p.x,scale*p.z));
    }
    points.clear();
    converted=true;
}


bool RouteMap::isConverted(){
    return this->converted;
}

std::list<cv::Point2f> RouteMap::GetRoute(cv::Point3f startPt){
    setScale(50);
    if(!converted){
        Convert2D(scale);
        
    }
    
    AStar astar;
    astar.mazeInit(_2Dpoints);
    std::cout <<"Maze Inited(Unit: m)"<<std::endl;
    PtA *start= new PtA(startPt.x,startPt.z);
    auto mazesize=astar.getMazeSize();
    auto ends=getPossibleEnd(Ow, mazesize);
    //std::cout <<"End Point got"<<std::endl;
    auto end=getFarthestPoint(Ow,getPossibleEnds(Ow,mazesize,15));
    PtA *ptEnd=new PtA(ends.x,ends.y);
    std::cout << "Start = " << endl << " (" << start->pt.x /(double)scale<<", "<<start->pt.y/(double)scale<<")"<< endl;
    std::cout << "End = " << endl << " (" << ptEnd->pt.x /(double)scale<<", "<<ptEnd->pt.y/(double)scale<<")"<< endl;
    std::list<PtA*> pathPtA=astar.getPath(*ptEnd,*start,true,5);
    pathPtA.reverse();
    //std::list<cv::Point2i> path;
    std::list<cv::Point2f> pathf;
    for(std::list<PtA*>::iterator it=pathPtA.begin(),ited=pathPtA.end();it!=ited;it++)
    {
        //path.push_back(cv::Point2i((*it)->pt.x,(*it)->pt.y));
        pathf.push_back(cv::Point2f((*it)->pt.x/(double)scale,(*it)->pt.y/(double)scale));
    }
    pathPtA.clear();
    pathf.push_front(cv::Point2f(startPt.x,startPt.z));
    pathf.push_back(cv::Point2f(ends.x,ends.y));
    std::cout <<"Path Size: "<<pathf.size()<<std::endl<<endl;
    return pathf;
}


void RouteMap::setCamCoodinate(const cv::Mat pos){
    pos.copyTo(Ow);
}

void RouteMap::setTwc(const cv::Mat _twc){
    _twc.copyTo(Twc);
    //Twc=_twc.clone();
}

cv::Point2f RouteMap::getPossibleEnd(cv::Mat &_Ow, std::vector<float> const &size)
{
    std::vector<float> p={0,0,2.5,1};
    cv::Mat z1=cv::Mat(p);
    //std::cout << "Twc = " << endl << " " << Twc << endl << endl;
    //std::cout << "z = " << endl << " " << z1 << endl << endl;
    cv::Mat dr=Twc*z1;
    //std::cout << "_Ow = " << endl << " " << _Ow.clone() << endl << endl;
    //std::cout << "Dr* = " << endl << " " << dr.rowRange(0,3).clone() << endl << endl;
    cv::Point3f cam(_Ow.clone());
    cv::Point2f start(cam.x,cam.z);
    cv::Point3f Dr(dr.rowRange(0,3).clone());
    cv::Point2f heading(2.0f*Dr.x,2.0f*Dr.z);
    //std::cout <<"OK here"<<std::endl;
    double deltay=heading.y-start.y;
    double deltax=heading.x-start.x;
    cv::Point2f end;
    if(deltax==0){
        end.x=heading.x;
        if(deltay>0)
        {
            end.y=size[2];

        }
        else
        {
            end.y=size[3];
        }
        return end;
    }else{
        double k=deltay/deltax;
        double dd=k*heading.x-heading.y;
        if(deltay!=0){
            if(deltax>0){
                cv::Point2f end1(size[0],(size[0])*k+dd);
                return end1;
            }
            else
            {
                cv::Point2f end12(size[1],(size[1])*k+dd);
                return end12;
            }
        }else{
            if(deltax>0){
                cv::Point2f end3(size[0],heading.y);
                return end3;
            }
            else
            {
                cv::Point2f end32(size[1],heading.y);
                return end32;
            }
            
        }
    }
    return cv::Point2f(0,0);
}

cv::Point2f RouteMap::getPossibleEnd(cv::Mat &_Ow, std::vector<float> const &size, int angle)
{
    double y=2.5*cos(M_PI*(angle/360.0)),x=2.5*sin(M_PI*(angle/360.0));
    std::vector<float> p={(float)x,0,(float)y,1};
    cv::Mat z1=cv::Mat(p);
    //std::cout << "Twc = " << endl << " " << Twc << endl << endl;
    //std::cout << "z = " << endl << " " << z1 << endl << endl;
    cv::Mat dr=Twc*z1;
    //std::cout << "_Ow = " << endl << " " << _Ow.clone() << endl << endl;
    //std::cout << "Dr* = " << endl << " " << dr.rowRange(0,3).clone() << endl << endl;
    cv::Point3f cam(_Ow.clone());
    cv::Point2f start(cam.x,cam.z);
    cv::Point3f Dr(dr.rowRange(0,3).clone());
    cv::Point2f heading(2.0f*Dr.x,2.0f*Dr.z);
    //std::cout <<"OK here"<<std::endl;
    double deltay=heading.y-start.y;
    double deltax=heading.x-start.x;
    cv::Point2f end;
    if(deltax==0){
        end.x=heading.x;
        if(deltay>0)
        {
            end.y=size[2];

        }
        else
        {
            end.y=size[3];
        }
        return end;
    }else{
        double k=deltay/deltax;
        double dd=k*heading.x-heading.y;
        if(deltay!=0){
            if(deltax>0){
                cv::Point2f end1(size[0],(size[0])*k+dd);
                return end1;
            }
            else
            {
                cv::Point2f end12(size[1],(size[1])*k+dd);
                return end12;
            }
        }else{
            if(deltax>0){
                cv::Point2f end3(size[0],heading.y);
                return end3;
            }
            else
            {
                cv::Point2f end32(size[1],heading.y);
                return end32;
            }
            
        }
    }
    return cv::Point2f(0,0);
}

void RouteMap::pathRecover(std::list<cv::Point2i> path)
{

}

void RouteMap::debug(std::string str)
{
    cout<<str<<endl;
}

void RouteMap::setScale(unsigned int _scale)
{
    scale=_scale;
}

std::list<cv::Point2f> RouteMap::getPossibleEnds(cv::Mat &_Ow, std::vector<float> &size,int step)
{
    cv::Point3f cam(_Ow.clone());
    cv::Point2f start(cam.x,cam.z);
    std::list<cv::Point2f> points;
    for(int i=-90;i<=90;i+=step)
    {
        cv::Point2f pt=getPossibleEnd(_Ow,size,i);
        if(pt.x!=start.x&&pt.y!=start.y)
        {
            points.push_back(pt);
        }
    }
    return points;
}

cv::Point2f RouteMap::getFarthestPoint(cv::Mat &_Ow, std::list<cv::Point2f> points)
{
    cv::Point3f cam(_Ow.clone());
    cv::Point2f start(cam.x,cam.z);
    cv::Point2f farPt(start.x,start.y);

    if(points.empty())
    {
        return start;
    }
    else
    {
        double distance=0;
        //calculate hamming distance;
        for(std::list<cv::Point2f>::iterator it=points.begin(),ited=points.end();it!=ited;it++)
        {
            double d=abs(it->x-start.x)+abs(it->y-start.y);
            if(d>distance)
            {
                distance=d;
                farPt.x=it->x;
                farPt.y=it->y;
            }

        }
        
    }
    return farPt;
    
}