#include <stdio.h>
#include <dirent.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ReadParams.h"

int getCameraParams(cv::Mat & cameraMatrix, cv::Mat & dst_cameraMatrix,
  cv::Mat & distCoeffs,std::string flag, double rx, double ry);
int getCameraParams_senyun(cv::Mat & cameraMatrix, cv::Mat & dst_cameraMatrix,
                    cv::Mat & distCoeffs,std::string flag, double rx, double ry,
                    double cx,double cy);

int main(int argc, const char * argv[]) {

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dst_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    std::string which_video = "10rear";
    std::string base_dir = "/home/lx/data/ceping_data/senyun/seq5/";
    std::string video_dir = "/home/lx/data/surround_line14_test/lednet/" + which_video + "/10rear.avi";
    std::string img_distorted_dir = base_dir + "/imgs/";
    std::string img_undistorted_dir = base_dir + "/dedist_imgs/";

    cv::VideoCapture cap(video_dir);
    cv::Size imageSize(640, 480);
    cv::Mat map_x, map_y;
    cv::Mat frame, dst;
    int n = 0;
    bool isVideo = 0;
    struct dirent *dirp;
    DIR *dir = opendir(img_distorted_dir.c_str());
    getCameraParams_senyun(cameraMatrix, dst_cameraMatrix,
                    distCoeffs, "ft", 1, 1,1,1); //left
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                         cameraMatrix, imageSize, CV_32FC1, map_x, map_y);
    if (isVideo)
    {
        while (cap.isOpened())
        {
            cap.read(frame);
            getCameraParams(cameraMatrix, dst_cameraMatrix,
                            distCoeffs, "ar", 1, 1); //left
            cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                                 cameraMatrix, imageSize, CV_32FC1, map_x, map_y);
            cv::remap(frame, dst, map_x, map_y, cv::INTER_LINEAR);

            cv::imwrite("/home/lx/data/surround_line14_test/lednet/" + which_video + "/img_undistorted/im_" + std::to_string(n) + ".jpg", dst);
            cv::imwrite("/home/lx/data/surround_line14_test/lednet/" + which_video + "/img_distorted/im_" + std::to_string(n) + ".jpg", frame);

            cv::imshow("frame", frame);
            cv::imshow("dst", dst);
            cv::waitKey(1);
            n++;
        }

    }
    else
    {
        while ((dirp = readdir(dir)) != NULL)
        {
            if (dirp->d_type == DT_REG)
            {
                std::string tname = dirp->d_name;
                std::cout<<"img name now propressed is = " << tname <<std::endl;
                frame = cv::imread(img_distorted_dir+tname);

                cv::remap(frame, dst, map_x, map_y, cv::INTER_LINEAR);

                cv::imwrite(img_undistorted_dir + tname, dst);

                cv::imshow("distort img", frame);
                cv::imshow("dst", dst);
                cv::waitKey(1);

            }
        }
    }


}



//    std::string imgBaseDir = "/home/lx/data_share/sengyun_test/src/";
//
//
//    std::string imgDstDir = "/home/lx/data_share/sengyun_test/dst/";
//    std::string label_png = "/home/lx/data/dis-test/label-png/";
//    std::string label_inv = "/home/lx/data/dis-test/label-inv/";
//    std::string label_inv_roi = "/home/lx/data/dis-test/label-inv-roi/";
//
//    DIR* dir = opendir(imgBaseDir.c_str());
//    cv::Mat src,labelPng;
//    cv::Mat dst;
//    cv::Mat view, rview, map_x, map_y;
//    cv::Size imageSize(640,480);
//    std::string::size_type pos;
//
//    std::string s_img ;
//    std::string sub ;
//    std::string imgDir,labelDir;
//
//    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
//    cv::Mat dst_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
//    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
//    while ((dirp = readdir(dir)) != NULL)
//    {
//      if (dirp->d_type == DT_REG)
//      {
//	s_img = dirp->d_name;
//	std::cout<< s_img <<std::endl;
//
//	int numof_ = s_img.find_first_of("_");
//	sub = s_img.substr(numof_-2,2);
//
//	imgDir = imgBaseDir + s_img; //src image dir
//	labelDir = label_png + s_img.substr(0,6)+".png"; //label-png dir
//	src = cv::imread(imgDir);
//	if (src.empty())
//	{
//	    std::cout<<"read src error!"<<std::endl;
//	    break;
//	}
//	else
//    {
//        std::cout<<"read src successed!"<<std::endl;
//    }
//    cv::imshow("src", src);
//	cv::waitKey(0);
//	//labelPng = cv::imread(labelDir, cv::IMREAD_GRAYSCALE);
//	getCameraParams_senyun(cameraMatrix,dst_cameraMatrix,distCoeffs, sub,1,1,3,3);
//	std::cout<<"dst_cameraMatrix"<<dst_cameraMatrix<<std::endl;
//	std::cout<<"distCoeffs"<<distCoeffs<<std::endl;
//
//	cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//	  dst_cameraMatrix,imageSize*3, CV_32FC1, map_x, map_y);
//	cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR);
//
//	//准备逆透视变换
//    cv::Mat img_ipm;
//    cv::Point2f AffinePoints0[4] = { cv::Point2f(100, 50), cv::Point2f(100, 390),
//                                     cv::Point2f(600, 50), cv::Point2f(600, 390) };
//    cv::Point2f AffinePoints1[4] = { cv::Point2f(200, 100), cv::Point2f(200, 330),
//                                     cv::Point2f(500, 50), cv::Point2f(600, 390) };
//    cv::Mat trans = cv::getPerspectiveTransform(AffinePoints0,AffinePoints1);
//    cv::warpPerspective(dst,img_ipm,trans,cv::Size(640*2,480*2)); //透视变换
//
//    std::cout<<"trans" << trans << std::endl;
//
//	std::cout<<"map_x"<<map_x(cv::Rect(0,0,5,5))<<std::endl;
//	std::cout<<"map_y"<<map_y(cv::Rect(0,0,5,5))<<std::endl;
//
//	cv::imshow("dst",dst);
//	//cv::imshow("img_ipm",img_ipm);
//	cv::waitKey(0);
//
//	}
//        else if (dirp->d_type == DT_DIR)
//	{
//	  ;
//        }
//    }
//    closedir(dir);
//    return 0;


int getCameraParams(cv::Mat & cameraMatrix, cv::Mat & dst_cameraMatrix,
  cv::Mat & distCoeffs,std::string flag, double rx, double ry)
{
  cv::Mat tcameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t_dst_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat tdistCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  if(flag == "ft") //left
  {
	tcameraMatrix.at<double>(0, 0) = 273.941;
	tcameraMatrix.at<double>(0, 2) = 328.962;
	tcameraMatrix.at<double>(1, 1) = 274.807;
	tcameraMatrix.at<double>(1, 2) = 287.867;

	t_dst_cameraMatrix.at<double>(0, 0) = 273.941*rx;
	t_dst_cameraMatrix.at<double>(0, 2) = 328.962;
	t_dst_cameraMatrix.at<double>(1, 1) = 274.807*ry;
	t_dst_cameraMatrix.at<double>(1, 2) = 287.867;

	tdistCoeffs.at<double>(0, 0) = -0.0436360;
	tdistCoeffs.at<double>(1, 0) = 0.014042;
	tdistCoeffs.at<double>(2, 0) = -0.0247863;
	tdistCoeffs.at<double>(3, 0) = 0.0131556;
	cameraMatrix = tcameraMatrix;
	dst_cameraMatrix = t_dst_cameraMatrix;
	distCoeffs = tdistCoeffs;
  }
  else if (flag == "ar") //rear
  {
    	tcameraMatrix.at<double>(0, 0) = 271.884;
	tcameraMatrix.at<double>(0, 2) = 337.451;
	tcameraMatrix.at<double>(1, 1) = 272.576;
	tcameraMatrix.at<double>(1, 2) = 238.858;

	t_dst_cameraMatrix.at<double>(0, 0) = 271.884*rx;
	t_dst_cameraMatrix.at<double>(0, 2) = 337.451;
	t_dst_cameraMatrix.at<double>(1, 1) = 272.576*ry;
	t_dst_cameraMatrix.at<double>(1, 2) = 238.858;

	tdistCoeffs.at<double>(0, 0) = -0.0395307;
	tdistCoeffs.at<double>(1, 0) = 0.00890713;
	tdistCoeffs.at<double>(2, 0) = -0.0233154;
	tdistCoeffs.at<double>(3, 0) = 0.010521;

	cameraMatrix = tcameraMatrix;
	dst_cameraMatrix = t_dst_cameraMatrix;
	distCoeffs = tdistCoeffs;
  }
  else if (flag == "nt") //front
  {
    	tcameraMatrix.at<double>(0, 0) = 267.444;
	tcameraMatrix.at<double>(0, 2) = 323.113;
	tcameraMatrix.at<double>(1, 1) = 267.617;
	tcameraMatrix.at<double>(1, 2) = 260.156;

	t_dst_cameraMatrix.at<double>(0, 0) = 267.444*rx;
	t_dst_cameraMatrix.at<double>(0, 2) = 323.113;
	t_dst_cameraMatrix.at<double>(1, 1) = 267.617*ry;
	t_dst_cameraMatrix.at<double>(1, 2) = 260.156;

	tdistCoeffs.at<double>(0, 0) = -0.041605;
	tdistCoeffs.at<double>(1, 0) = 0.000133707;
	tdistCoeffs.at<double>(2, 0) = -0.0118289;
	tdistCoeffs.at<double>(3, 0) = 0.00706186;
	cameraMatrix = tcameraMatrix;
	dst_cameraMatrix = t_dst_cameraMatrix;
	distCoeffs = tdistCoeffs;
  }
  else if (flag == "ht") //right
  {
    std::cout<<flag<<std::endl;
        tcameraMatrix.at<double>(0, 0) = 274.624;
	tcameraMatrix.at<double>(0, 2) = 324.817;
	tcameraMatrix.at<double>(1, 1) = 274.019;
	tcameraMatrix.at<double>(1, 2) = 243.932;

	t_dst_cameraMatrix.at<double>(0, 0) = 274.624*rx;
	t_dst_cameraMatrix.at<double>(0, 2) = 324.817;
	t_dst_cameraMatrix.at<double>(1, 1) = 274.019*ry;
	t_dst_cameraMatrix.at<double>(1, 2) = 243.932;

	tdistCoeffs.at<double>(0, 0) = -0.0752909;
	tdistCoeffs.at<double>(1, 0) = 0.0671028;
	tdistCoeffs.at<double>(2, 0) = -0.092907;
	tdistCoeffs.at<double>(3, 0) = 0.0436096;

	cameraMatrix = tcameraMatrix;
	dst_cameraMatrix = t_dst_cameraMatrix;
	distCoeffs = tdistCoeffs;
  }
  else
  {
    return -1;
  }
}

int getCameraParams_senyun(cv::Mat & cameraMatrix, cv::Mat & dst_cameraMatrix,
                    cv::Mat & distCoeffs,std::string flag, double rx, double ry,
           double cx, double cy)
{
    cv::Mat tcameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t_dst_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat tdistCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    if(flag == "ft") //left
    {
        tcameraMatrix.at<double>(0, 0) = 155.573;
        tcameraMatrix.at<double>(0, 2) = 324.91;
        tcameraMatrix.at<double>(1, 1) = 155.704;
        tcameraMatrix.at<double>(1, 2) = 251.636;
        t_dst_cameraMatrix.at<double>(0, 0) = 155.573*rx;
        t_dst_cameraMatrix.at<double>(0, 2) = 324.91 * cx;
        t_dst_cameraMatrix.at<double>(1, 1) = 155.704*ry;
        t_dst_cameraMatrix.at<double>(1, 2) = 251.636 * cy;
        tdistCoeffs.at<double>(0, 0) = 0.0756011;
        tdistCoeffs.at<double>(1, 0) = 0.0486787;
        tdistCoeffs.at<double>(2, 0) = -0.0317964;
        tdistCoeffs.at<double>(3, 0) = 0.00533516;
        cameraMatrix = tcameraMatrix;
        dst_cameraMatrix = t_dst_cameraMatrix;
        distCoeffs = tdistCoeffs;
    }
    else if (flag == "ar") //rear
    {
        tcameraMatrix.at<double>(0, 0) = 157.003;
        tcameraMatrix.at<double>(0, 2) = 323.119;
        tcameraMatrix.at<double>(1, 1) = 157.082;
        tcameraMatrix.at<double>(1, 2) = 238.298;
        t_dst_cameraMatrix.at<double>(0, 0) = 157.003*rx;
        t_dst_cameraMatrix.at<double>(0, 2) = 323.119*cx;
        t_dst_cameraMatrix.at<double>(1, 1) = 157.082*ry;
        t_dst_cameraMatrix.at<double>(1, 2) = 238.298*cy;
        tdistCoeffs.at<double>(0, 0) = 0.0773366;
        tdistCoeffs.at<double>(1, 0) = 0.0272749;
        tdistCoeffs.at<double>(2, 0) = -0.00766821;
        tdistCoeffs.at<double>(3, 0) = -0.00325986;
        cameraMatrix = tcameraMatrix;
        dst_cameraMatrix = t_dst_cameraMatrix;
        distCoeffs = tdistCoeffs;
    }
    else if (flag == "nt") //front
    {
        tcameraMatrix.at<double>(0, 0) = 156.036;
        tcameraMatrix.at<double>(0, 2) = 326.52;
        tcameraMatrix.at<double>(1, 1) = 156.21;
        tcameraMatrix.at<double>(1, 2) =  230.456;
        t_dst_cameraMatrix.at<double>(0, 0) = 156.036*rx;
        t_dst_cameraMatrix.at<double>(0, 2) = 326.52 * cx;
        t_dst_cameraMatrix.at<double>(1, 1) = 156.21*ry;
        t_dst_cameraMatrix.at<double>(1, 2) = 230.456 * cy;
        tdistCoeffs.at<double>(0, 0) = 0.0738011;
        tdistCoeffs.at<double>(1, 0) = 0.0513934;
        tdistCoeffs.at<double>(2, 0) = -0.0334965;
        tdistCoeffs.at<double>(3, 0) = 0.00559905;
        cameraMatrix = tcameraMatrix;
        dst_cameraMatrix = t_dst_cameraMatrix;
        distCoeffs = tdistCoeffs;
    }
    else if (flag == "ht") //right
    {
        std::cout<<flag<<std::endl;
        tcameraMatrix.at<double>(0, 0) = 155.548;
        tcameraMatrix.at<double>(0, 2) = 316.124;
        tcameraMatrix.at<double>(1, 1) = 155.764;
        tcameraMatrix.at<double>(1, 2) = 215.052;
        t_dst_cameraMatrix.at<double>(0, 0) = 155.548*rx;
        t_dst_cameraMatrix.at<double>(0, 2) = 316.124*cx;
        t_dst_cameraMatrix.at<double>(1, 1) = 155.764*ry;
        t_dst_cameraMatrix.at<double>(1, 2) = 215.052*cy;
        tdistCoeffs.at<double>(0, 0) = 0.0836834;
        tdistCoeffs.at<double>(1, 0) = 0.0339519;
        tdistCoeffs.at<double>(2, 0) = -0.0207494;
        tdistCoeffs.at<double>(3, 0) = 0.00256549;
        cameraMatrix = tcameraMatrix;
        dst_cameraMatrix = t_dst_cameraMatrix;
        distCoeffs = tdistCoeffs;
    }
    else
    {
        return -1;
    }
    return 1;
}
