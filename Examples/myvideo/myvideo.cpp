#include <System.h>
//https://blog.csdn.net/qq_33414553/article/details/80834082
#include <string>
#include <chrono> // for time stamp
#include <iostream>
#include <opencv2/core/core.hpp>
using namespace std;
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./myvideo.yaml";
string vocFile = "./Vocabulary/ORBvoc.bin";
// 视频文件
string videoFile = "./myvideo.mp4";
int main(int argc, char **argv)
{
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);
    // 获取视频图像
    cv::VideoCapture cap(videoFile);
    // change to 1 if you want to use USB camera.
    // 记录系统时间
    auto start = chrono::system_clock::now();
    for (;;)
    {
        cv::Mat frame;
        cap >> frame;
        // 读取相机数据
        if (frame.data == nullptr)
            break;
        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(640, 360));
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame_resized, double(timestamp.count()) / 1000.0);
        // press to go to next frame
        cv::waitKey(30);
    }
    // Save map
    //SLAM.SaveMap("Slam_latest_Map.bin");
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // stop all threads
    SLAM.Shutdown();
    return 0;
}
