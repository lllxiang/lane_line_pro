#include<opencv2/opencv.hpp>
#include<fstream>
#include "ReadParams.h"

#define video_calibrate_perspective_Mat 1
#define camera_calibrate_perspective_Mat 0

/*****************************    coordinate system transform       *******************************/

 bool findCorners(const cv::Mat img, cv::Size board_size, std::vector<cv::Point2f> &corners)
{
    cv::Mat imageGray;
    cv::cvtColor(img, imageGray, CV_RGB2GRAY);
    bool patternfound = cv::findChessboardCorners(img, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
        cv::CALIB_CB_FAST_CHECK);
    if (!patternfound)
        return false;
    else
    {
        cv::cornerSubPix(imageGray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        return true;
    }
}
std::vector<double> readDistortionParams(const std::string distortionFileName)
{
    std::vector<double> distortion;
    std::ifstream in;
    in.open(distortionFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open distortionFile!" << std::endl;
        exit(EXIT_FAILURE);
    }
    for (double i; in >> i;)
    {
        distortion.push_back(i);
    }
    in.close();
    return distortion;
}

cv::Mat readIntrinsicParams(const std::string intrinsicFileName)
{
    cv::Mat intrinsic(cv::Size(3, 3), CV_64FC1);
    std::ifstream in;
    std::cout<<"intrinsicFileName:"<<intrinsicFileName<<std::endl;
    in.open(intrinsicFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open intrinsicFile!" << std::endl;
        exit(EXIT_FAILURE);
    }

    intrinsic.setTo(0);
    in >> intrinsic.at<double>(0, 0);
    in >> intrinsic.at<double>(0, 2);
    in >> intrinsic.at<double>(1, 1);
    in >> intrinsic.at<double>(1, 2);
    intrinsic.at<double>(2, 2) = 1;
    in.close();
    return intrinsic;
}
void readExtrinsic(const std::string extrinsicFileName, cv::Mat &rotation_vectors, cv::Mat &translation_vectors)
{
    std::ifstream in;
    in.open(extrinsicFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open extrinsicFile!" << std::endl;
        exit(EXIT_FAILURE);
    }
    in >> rotation_vectors.at<double>(0, 0);
    in >> rotation_vectors.at<double>(1, 0);
    in >> rotation_vectors.at<double>(2, 0);

    in >> translation_vectors.at<double>(0, 0);
    in >> translation_vectors.at<double>(1, 0);
    in >> translation_vectors.at<double>(2, 0);
    in.close();

}

void readPerspectiveParams(const std::string perspectiveFileName, cv::Mat &perspectiveMat,cv::Mat &shifPerspectiveMat)
{
    //cv::Mat perspectiveMat(cv::Size(3, 3), CV_64FC1);
    std::ifstream in;

    in.open(perspectiveFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open perspectiveFile!" << std::endl;
        exit(EXIT_FAILURE);
    }
    in >> perspectiveMat.at<double>(0, 0);
    in >> perspectiveMat.at<double>(0, 1);
    in >> perspectiveMat.at<double>(0, 2);
    in >> perspectiveMat.at<double>(1, 0);
    in >> perspectiveMat.at<double>(1, 1);
    in >> perspectiveMat.at<double>(1, 2);
    in >> perspectiveMat.at<double>(2, 0);
    in >> perspectiveMat.at<double>(2, 1);
    in >> perspectiveMat.at<double>(2, 2);
    in >> shifPerspectiveMat.at<double>(0, 0);
    in >> shifPerspectiveMat.at<double>(0, 1);
    in >> shifPerspectiveMat.at<double>(0, 2);
    in >> shifPerspectiveMat.at<double>(1, 0);
    in >> shifPerspectiveMat.at<double>(1, 1);
    in >> shifPerspectiveMat.at<double>(1, 2);
    in >> shifPerspectiveMat.at<double>(2, 0);
    in >> shifPerspectiveMat.at<double>(2, 1);
    in >> shifPerspectiveMat.at<double>(2, 2);
    in.close();
 }


