
#include<opencv2/opencv.hpp>
#include<iostream>
#include<fstream>
#include<string>


extern std::vector<cv::Point> mousePoint;
extern cv::Mat mapx;
extern cv::Mat mapy;


    
std::vector<double> readDistortionParams(const std::string distortionFileName);
cv::Mat readIntrinsicParams(const std::string intrinsicFileName);
void readExtrinsic(const std::string extrinsicFileName, cv::Mat &rotation_vectors, cv::Mat &translation_vectors);

void readPerspectiveParams(const std::string perspectiveFileName, cv::Mat &perspectiveMat,cv::Mat &shifPerspectiveMat);
void on_mouse(int event, int x, int y, int flag, void *param);
bool perspectiveTransCalibration();
void moving_mouse(int event, int x, int y, int flags, void* ustc);
bool findCorners(const cv::Mat img, cv::Size board_size, std::vector<cv::Point2f> &corners);
void  computeWorldcoordWithZ(const std::vector<cv::Point2f>& imagePoints,
                                          std::vector<cv::Point2f>& objectPoints,
                                          const cv::Mat&  rvec,
                                          const cv::Mat&  tvec,
                                          const cv::Mat&  K);
