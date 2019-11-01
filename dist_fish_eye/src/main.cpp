#include <stdio.h>
#include <dirent.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

int getCameraParams(cv::Mat & cameraMatrix, cv::Mat & dst_cameraMatrix,
  cv::Mat & distCoeffs,std::string flag);

int main(int argc, const char * argv[])
{
    struct dirent *dirp;
    std::string src_img_dir = "../data/src_image/";
    std::string undisted_img_dir = "../data/undisted_image/";
    std::string undisted_img_inv_dir = "../data/undisted_image_inv/";
    std::string undisted_img_inv_roi_dir = "../data/undisted_image_inv_crop/";

	cv::Mat src,labelPng;
	cv::Mat dst;
	cv::Mat view, rview, map_x, map_y;
	cv::Size imageSize(640,480);

    DIR* dir = opendir(src_img_dir.c_str());
    std::string::size_type pos;
    std::string s_img ;
    std::string sub ;
    std::string imgDir,labelDir;

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dst_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    while ((dirp = readdir(dir)) != NULL)
    {
      if (dirp->d_type == DT_REG) 
      {
		s_img = dirp->d_name;
		imgDir = src_img_dir + s_img; //src image dir
		src = cv::imread(imgDir);
		getCameraParams(cameraMatrix,dst_cameraMatrix,distCoeffs, "ft");

		cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
		                                dst_cameraMatrix,imageSize*3, CV_32FC1, map_x, map_y);
		cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR);

		map_x.convertTo(map_x, CV_16UC1, 1, 0);
		map_y.convertTo(map_y, CV_16UC1, 1, 0);
		map_x.convertTo(map_x, CV_32FC1, 1, 0);
		map_y.convertTo(map_y, CV_32FC1, 1, 0);
		cv::Mat inv_mapx, inv_mapy,  inv_dst, inv_label;
		inv_mapx = cv::Mat::zeros(map_x.size(), CV_32FC1);
		inv_mapy = cv::Mat::zeros(map_y.size(), CV_32FC1);
		for(int i=0; i<map_x.rows;i++)
		{
		  for(int j=0;j<map_x.cols;j++)
		  {
			int x = int(map_x.at<float>(i,j));
			int y = int(map_y.at<float>(i,j));
			inv_mapx.at<float>(y,x) = j;
			inv_mapy.at<float>(y,x) = i;
		  }
		}

		dst.at<cv::Vec3b>(0,0)[0] = 0; dst.at<cv::Vec3b>(0,0)[1] = 0; dst.at<cv::Vec3b>(0,0)[2] = 0;
		cv::remap(dst, inv_dst, inv_mapx, inv_mapy, cv::INTER_CUBIC,
		  2, cv::Scalar(0,0, 0));

		cv::imshow("src",src); //src image
		cv::imshow("dst",dst); //dist image
		cv::imshow("inv_dst", inv_dst); //inv dist image

		cv::imwrite(undisted_img_dir+s_img, dst);
		cv::imwrite(undisted_img_inv_dir + s_img, inv_dst);
		cv::imwrite(undisted_img_inv_roi_dir + s_img, inv_dst(cv::Rect(0,0,640,480)));
		cv::waitKey(0);
        } 
        else if (dirp->d_type == DT_DIR)
        {
        	;
        }
    }
    closedir(dir);
    return 0;
}
 
int getCameraParams(cv::Mat & cameraMatrix, cv::Mat & dst_cameraMatrix,
  cv::Mat & distCoeffs,std::string flag)
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

	t_dst_cameraMatrix.at<double>(0, 0) = 273.941;
	t_dst_cameraMatrix.at<double>(0, 1) = 0;
	t_dst_cameraMatrix.at<double>(0, 2) = 328.962*3;
	t_dst_cameraMatrix.at<double>(1, 1) = 274.807;
	t_dst_cameraMatrix.at<double>(1, 2) = 287.867*3;

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

	t_dst_cameraMatrix.at<double>(0, 0) = 271.884;
	t_dst_cameraMatrix.at<double>(0, 2) = 337.451;
	t_dst_cameraMatrix.at<double>(1, 1) = 272.576;
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

	t_dst_cameraMatrix.at<double>(0, 0) = 267.444;
	t_dst_cameraMatrix.at<double>(0, 2) = 323.113;
	t_dst_cameraMatrix.at<double>(1, 1) = 267.617;
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

	t_dst_cameraMatrix.at<double>(0, 0) = 274.624;
	t_dst_cameraMatrix.at<double>(0, 2) = 324.817;
	t_dst_cameraMatrix.at<double>(1, 1) = 274.019;
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
