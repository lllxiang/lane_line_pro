#include<opencv2/opencv.hpp>
#include <iostream>
#include "computeWorldcoordWithZ.h"
#include "ReadParams.h"
#include <dirent.h>
struct line_info
{
    int line_type; //X型=1， Y=0
    cv::Point left;
    cv::Point right;
    std::vector<cv::Point> point_fit; //拟合使用的点
    cv::Mat A1; //多项式系数
    cv::Mat A2;
};
std::string distortionFileName;
std::string intrinsicFileName;
std::string perspectiveFileName;
std::string extrinsicFileName;

const double car_len = 500.5;    //自车长
const double car_weight = 193.2; //车宽
const double car_zhouju = 187.5;
const double view_dis = 1000.0; //可视距离 800cm

//输入：方向标识，type,原图，
//输出，IPM图
int ipm_trans(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp);
//威视相机外参为止情况下，通过手工选点，估计IPM m矩阵
int ipm_trans_weishi(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp);
bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id);
//原图坐标->去畸变图坐标
bool src2dist(std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
              cv::Mat & mapx, cv::Mat & mapy);
bool read_cam_params(std::string &d);


bool detect_line(cv::Mat & line_pred, std::vector<line_info> & lines);

bool fit_one_line(std::vector<cv::Point> &c, line_info & lines);

bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A);
std::vector<int> GetFlags(int a[],int length)
{
    std::vector<int> vec;
    int neighbour[]={1,2,4,8,16,32,64,128,1,2,4,8,16,32,64};
    for(int i=0;i<length;i++)
    {
        for(int j=0;j<8;j++)
        {
            int sum=0;
            for(int k=j;k<=j+a[i];k++)
                sum+=neighbour[k];
            vec.push_back(sum);
            std::cout<<sum<<" ";
        }
    }
    std::cout<<std::endl;
    return vec;
}
void skeleton(cv::Mat &Input) //Input-binary image
{
    int a0[]={1,2,3,4,5,6};
    int a1[]={2};
    int a2[]={2,3};
    int a3[]={2,3,4};
    int a4[]={2,3,4,5};
    int a5[]={2,3,4,5,6};
    std::vector<int> A0=GetFlags(a0,6);

    std::vector<int> A1=GetFlags(a1,1);

    std::vector<int> A2=GetFlags(a2,2);
    std::vector<int> A3=GetFlags(a3,3);
    std::vector<int> A4=GetFlags(a4,4);
    std::vector<int> A5=GetFlags(a5,5);
    std::vector<cv::Point2i> border;
    bool modify=true;
    int neighbour[3][3]={
            {128,1,2},
            {64,0,4},
            {32,16,8}
    };
    int row=Input.rows;
    int col=Input.cols;
    while(modify)
    {
        modify=false;
        // flag the border Pharse 0
        for(int m=1;m<row-1;++m)
        {
            for(int n=1;n<col-1;++n)
            {
                int weight=0;
                for(int j=-1;j<=1;++j)
                {
                    for(int k=-1;k<=1;k++)
                    {
                        weight+=neighbour[j+1][k+1]*Input.at<uchar>(m+j,n+k);
                    }
                }
                if(std::find(A0.begin(),A0.end(),weight)!=A0.end())
                    border.push_back(cv::Point2i(m,n));
            }
        }
        //Pharse 1
        std::vector<cv::Point2i>::iterator first=border.begin();
        while(first!=border.end())
        {
            int weight=0;
            for(int j=-1;j<=1;++j)
            {
                for(int k=-1;k<=1;k++)
                {
                    weight+=neighbour[j+1][k+1]*Input.at<uchar>((*first).x+j,(*first).y+k);
                }
            }
            if(std::find(A1.begin(),A1.end(),weight)!=A1.end())
            {
                Input.at<uchar>((*first).x,(*first).y)=0;
                first=border.erase(first);
            }
            else
                ++first;
        }
        //Pharse2
        first=border.begin();
        while(first!=border.end())
        {
            int weight=0;
            for(int j=-1;j<=1;++j)
            {
                for(int k=-1;k<=1;k++)
                {
                    weight+=neighbour[j+1][k+1]*Input.at<uchar>((*first).x+j,(*first).y+k);
                }
            }
            if(std::find(A2.begin(),A2.end(),weight)!=A2.end())
            {
                Input.at<uchar>((*first).x,(*first).y)=0;
                first=border.erase(first);
            }
            else
                ++first;
        }
        //Pharse3
        first=border.begin();
        while(first!=border.end())
        {
            int weight=0;
            for(int j=-1;j<=1;++j)
            {
                for(int k=-1;k<=1;k++)
                {
                    weight+=neighbour[j+1][k+1]*Input.at<uchar>((*first).x+j,(*first).y+k);
                }
            }
            if(std::find(A3.begin(),A3.end(),weight)!=A3.end())
            {
                Input.at<uchar>((*first).x,(*first).y)=0;
                first=border.erase(first);
            }
            else
                ++first;
        }
        //Pharse4
        first=border.begin();
        while(first!=border.end())
        {
            int weight=0;
            for(int j=-1;j<=1;++j)
            {
                for(int k=-1;k<=1;k++)
                {
                    weight+=neighbour[j+1][k+1]*Input.at<uchar>((*first).x+j,(*first).y+k);
                }
            }
            if(std::find(A4.begin(),A4.end(),weight)!=A4.end())
            {
                Input.at<uchar>((*first).x,(*first).y)=0;
                first=border.erase(first);
            }
            else
                ++first;
        }
        //Pharse5
        first=border.begin();
        while(first!=border.end())
        {
            int weight=0;
            for(int j=-1;j<=1;++j)
            {
                for(int k=-1;k<=1;k++)
                {
                    weight+=neighbour[j+1][k+1]*Input.at<uchar>((*first).x+j,(*first).y+k);
                }
            }
            if(std::find(A5.begin(),A5.end(),weight)!=A5.end())
            {
                Input.at<uchar>((*first).x,(*first).y)=0;
                first=border.erase(first);
                modify=true;
            }
            else
                ++first;
        }
        //Pharse6
        border.clear();
    }
    for(int m=1;m<row-1;++m)
    {
        for(int n=1;n<col-1;++n)
        {
            int weight=0;
            for(int j=-1;j<=1;++j)
            {
                for(int k=-1;k<=1;k++)
                {
                    weight+=neighbour[j+1][k+1]*Input.at<uchar>(m+j,n+k);
                }
            }
            if(std::find(A0.begin(),A0.end(),weight)!=A0.end())
                Input.at<uchar>(m,n)=0;;
        }
    }

}
bool polyfit(std::vector<cv::Point>& in_point, int n, cv::Mat &mat_k);

int main()
{
    std::cout <<"aaa"<< std::endl;
    cv::Mat im = cv::imread("/home/lx/data/1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat im_c = cv::Mat::zeros(im.size(), CV_8UC3);
    cv::bitwise_not(im,im);

    //find contours
    std::vector<std::vector<cv::Point>> contours; //contours[0] is the first contours
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(im, contours, hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_SIMPLE);

    cv::Mat A, A2;
    std::vector<cv::Point> points_fitted , points_fitted2;
    std::vector<cv::Point> point2;
    for(int i=0;i<contours[0].size();i++)
    {
        point2.push_back(cv::Point(contours[0][i].y, contours[0][i].x));
    }
    polynomial_curve_fit(contours[0], 3, A);
    polynomial_curve_fit(point2, 3, A2);



    for (int i = 0; i < contours[0].size(); i++)
    {
        cv::circle(im_c,contours[0][i],2,cv::Scalar(255,0,0),1,8,0);
    }

    for (int x = 0; x < 640; x++)
    {
        double y = A.at<double>(0,0) + A.at<double>(0,1) * x +
                A.at<double>(0,2)*std::pow(x, 2) + A.at<double>(0,3)*std::pow(x, 3);
        points_fitted.push_back(cv::Point(x, y));
    }
    cv::polylines(im_c, points_fitted, false, cv::Scalar(0, 0, 255), 1, 8, 0);

    for (int y = 0; y < 480; y++)
    {
        double x = A2.at<double>(0,0) + A2.at<double>(0,1) * y +
                    A2.at<double>(0,2)*std::pow(y, 2) + A2.at<double>(0,3)*std::pow(y, 3);
        points_fitted2.push_back(cv::Point(x, y));
    }
    cv::polylines(im_c, points_fitted2, false, cv::Scalar(255, 0, 255), 1, 8, 0);


    cv::imshow("im", im);
    cv::imshow("im_c", im_c);
    cv::waitKey(0);
    return 0;
}

bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id)
{
    for(int h=0;h<src.rows; ++h)
    {
        for(int w=0;w<src.cols; ++w)
        {
            if (src.at<uchar>(h,w) == id )
            {
                dst.at<uchar>(h,w) = 1;
            }
            else
            {
                dst.at<uchar>(h,w) = 0;
            }
        }
    }
    return true;
}
bool src2dist(std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
              cv::Mat & mapx, cv::Mat & mapy)
{
    cv::Mat inv_mapx, inv_mapy,  inv_dst, inv_label;

    mapx.convertTo(mapx, CV_16UC1, 1, 0);
    mapy.convertTo(mapy, CV_16UC1, 1, 0);
    mapx.convertTo(mapx, CV_32FC1, 1, 0);
    mapy.convertTo(mapy, CV_32FC1, 1, 0);
    inv_mapx = cv::Mat::zeros(mapx.size(), CV_32FC1);
    inv_mapy = cv::Mat::zeros(mapy.size(), CV_32FC1);

    for(int i=0; i<mapx.rows;i++)
    {
        for(int j=0;j<mapx.cols;j++)
        {
            int x = int(mapx.at<float>(i,j));
            int y = int(mapy.at<float>(i,j));
            inv_mapx.at<float>(y,x) = j;
            inv_mapy.at<float>(y,x) = i;
        }
    }

    std::vector<cv::Point2f> dst_dist;
    for(int i=0;i<4;i++)
    {
        int x = src[i].x;
        int y = src[i].y;
        cv::Point2f tmp;
        tmp.x = inv_mapx.at<float>(y,x);
        tmp.y = inv_mapy.at<float>(y,x);
        dst.push_back(tmp);
    }
    return 1;
}
int ipm_trans_weishi(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp)
{

    std::vector<cv::Point2f> dst_dist; //俯视图 2D
    std::vector<cv::Point2f> wordPoints;

    //wordPoints.push_back(cv::Point2f());

    cv::Mat m = cv::getPerspectiveTransform(dst_dist,wordPoints);
    //cv::warpPerspective(img_jpg, warp, m, ipm_size);

    return 1;

}

int ipm_trans(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp)
{
    read_cam_params(driection);

    double pp = 1;
    double pk = 0.9;
    cv::Mat img_dist;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat cameraMatrix2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs(cv::Size(1,4),CV_64F); //Size(w,h)
    std::vector<double> distortion;
    std::cout<<"intrinsicFileName:"<<intrinsicFileName<<std::endl;
    cameraMatrix = readIntrinsicParams(intrinsicFileName);
    distortion = readDistortionParams(distortionFileName);
    cameraMatrix2.at<double>(0, 0) = cameraMatrix.at<double>(0, 0)*pk;
    cameraMatrix2.at<double>(0, 2) = cameraMatrix.at<double>(0, 2)*pp;
    cameraMatrix2.at<double>(1, 1) = cameraMatrix.at<double>(1, 1)*pk;
    cameraMatrix2.at<double>(1, 2) = cameraMatrix.at<double>(1, 2)*pp;
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    cv::Mat R; //旋转矩阵
    cv::Mat rvecs(cv::Size(1,3),CV_64F); //旋转向量
    cv::Mat tvecs(cv::Size(1,3),CV_64F); //平移矩阵
    readExtrinsic(extrinsicFileName, rvecs, tvecs);
    std::cout<< rvecs << std::endl;
    cv::Rodrigues(rvecs, R);
    std::vector<cv::Point3f> wordPoints3D;  //俯视图 3D temp
    std::vector<cv::Point2f> wordPoints2D; //俯视图 2D
    std::vector<cv::Point2f> distImgPoint; //去畸变图上的点
    std::vector<cv::Point2f> srcImgPoint;   //畸变图上的点
    std::vector<double> Z; Z.push_back(0.0); Z.push_back(0.0); Z.push_back(0.0); Z.push_back(0.0);
    srcImgPoint.push_back(cv::Point2f(108.0,327.0));
    srcImgPoint.push_back(cv::Point2f(468.0,366.0));
    srcImgPoint.push_back(cv::Point2f(449.0,163.0));
    srcImgPoint.push_back(cv::Point2f(143.0,170.0));
    std::vector<cv::Point2f> srcImgPoint_calu_w = srcImgPoint; //srcImgPoint_calu_w 在computeWorldcoordWithZ
    //中会被改变值，因此准备一个备份

    std::cout<< "旋转向量" << rvecs << std::endl;
    std::cout<< "旋转矩阵" << R << std::endl;
    std::cout<< "distCoeffs" << distCoeffs << std::endl;

    computeWorldcoordWithZ (wordPoints3D, srcImgPoint_calu_w, Z,R,
                            tvecs, cameraMatrix, distCoeffs);
    //确定 畸变图像点 与 俯视图 点对应关系
    for (int i=0; i<4; i++)
    {
        wordPoints2D.push_back(cv::Point2f(wordPoints3D[i].x,wordPoints3D[i].y));
        std::cout<< "wordPoints2D:" << wordPoints2D[i] << std::endl;
    }
    std::cout<< "distCoeffs" << distCoeffs.at<double>(3, 0) << std::endl;
    std::cout<< "cameraMatrix" << cameraMatrix << std::endl;

    //图像去畸变
    cv::Size imageSize(640*pp, 480*pp);
    cv::Mat mapx(imageSize, CV_32FC1);
    cv::Mat mapy(imageSize, CV_32FC1);
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs,  cv::Mat(), cameraMatrix2, imageSize, CV_32FC1, mapx, mapy);
    cv::remap(img_jpg, img_dist, mapx, mapy, cv::INTER_NEAREST);
    //求去畸变图像对应的点坐标
    src2dist(srcImgPoint, distImgPoint, mapx, mapy);

    //根据去畸变图点坐标和俯视图点坐标求M
    cv::Size ipm_size;
    for (int i=0; i<4; i++)
    {
        std::cout<<"distImgPoint in wh"<< distImgPoint[i] <<std::endl;
        std::cout<<"wordPoints2D in xy"<< wordPoints2D[i] <<std::endl;
        if(driection == "ft")
        {
            //left
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + (car_len-car_zhouju)/2 + 106))/2;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis - 103))/2;
            ipm_size = cv::Size((view_dis*2 + car_len)/2,  (view_dis+car_weight/2)/2);
        }
        else if (driection == "ar")
        {
            //rear
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + 50))/2;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis + (car_len-car_zhouju)/2 - 210))/2;
            ipm_size = cv::Size((view_dis*2 + car_weight)/2,  (view_dis + car_len/2)/2);
        }
        std::cout<<"wordPoints2D in wh"<< wordPoints2D[i] <<std::endl;
    }
    if (type)
    {//矫正图
        cv::Mat m = cv::getPerspectiveTransform(distImgPoint, wordPoints2D);
        cv::warpPerspective(img_jpg, warp, m, ipm_size);
    }
    else
    {
        cv::Mat m = cv::getPerspectiveTransform(distImgPoint, wordPoints2D);
        cv::warpPerspective(img_dist, warp, m, ipm_size);
    }
    return 1;

}
bool read_cam_params(std::string &d)
{
    std::string fish;
    std::string cam = "weishi";
    std::cout<<"d"<<d<<std::endl;
    if (d=="ar")
    {
        fish = "fish2";
    }
    else if (d == "ft")
    {
        fish = "fish4";
    }
    // 读取相机参数
    distortionFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/distortion.txt";
    intrinsicFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/intrinsic.txt";
    perspectiveFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/perspectiveTransformCalibration.txt";
    extrinsicFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/extrinsic.txt";
    return 1;
}

bool detect_line(cv::Mat & line_pred, std::vector<line_info> & lines)
{
    std::cout<<"detect_line"<<std::endl;
    double min_s = 30; //面积阈值
    //find contours
    std::vector<std::vector<cv::Point>> contours; //contours[0] is the first contours
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(line_pred, contours, hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_SIMPLE);
    if(contours.size() == 0)
    {
        std::cout<<"No any line in image."<<std::endl;
    }else if(contours.size() == 1) //只有一个轮廓，滤波后直接拟合
    {
        line_info one_line;
        int c_s = cv::contourArea(contours[0]); //find contours
        if (c_s > min_s)
        {
            //拟合.
            std::cout<<"is fitting..."<<std::endl;
            //contours[0] ->  params.
            cv::Mat A;
            fit_one_line(contours[0], one_line);

            std::cout<< "A" << one_line.A1 << std::endl;
            lines.push_back(one_line);
        }
        else
        {
            contours.pop_back();
            std::cout<<"No any line in image."<<std::endl;
        }
        //int cl = cv::arcLength(contours[0]); //calu contours length
    } else  //轮廓大于一个，滤波->聚类后再拟合
    {

    }
    return 1;
}
bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
    //Number of key points
    int N = key_point.size();
    //构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                                     std::pow(key_point[k].x, i + j);
            }
        }
    }

    //构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                                 std::pow(key_point[k].x, i) * key_point[k].y;
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //求解矩阵A
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}



bool polyfit(std::vector<cv::Point>& in_point, int n, cv::Mat &mat_k)
{
    int size = in_point.size();
    //所求未知数个数
    int x_num = n + 1;
    //构造矩阵U和Y
    cv::Mat mat_u(size, x_num, CV_64F);
    cv::Mat mat_y(size, 1, CV_64F);

    for (int i = 0; i < mat_u.rows; ++i)
        for (int j = 0; j < mat_u.cols; ++j)
        {
            mat_u.at<double>(i, j) = pow(in_point[i].x, j);
        }

    for (int i = 0; i < mat_y.rows; ++i)
    {
        mat_y.at<double>(i, 0) = in_point[i].y;
    }
    //矩阵运算，获得系数矩阵K
    //cv::Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
    std::cout <<"mat_k"<< mat_k << std::endl;
    return 1;
}


bool fit_one_line(std::vector<cv::Point> &c, line_info & lines)
{
    //输入 一组无序点集，分别拟合"X"型及"Y"型五次多项式曲线。
    //并选取误差最小的一组参数，作为单白线的曲线参数返回。
    std::vector<cv::Point> c2;
    for(int i=0;i<c.size();i++)
    {
        c2.push_back(cv::Point(c[i].y, c[i].x));
    }

    cv::Mat A1, A2;
    polynomial_curve_fit(c,5, A1); //X型拟合
    polynomial_curve_fit(c2,5, A2); //Y型拟合

    double s1=0, s2=0;
    for(int i = 0; i<c.size();i+=5)
    {
        int x =c[i].x;
        double y1 = A1.at<double>(0,0) + A1.at<double>(0,1) * x +
                    A1.at<double>(0,2)*std::pow(x, 2) + A1.at<double>(0,3)*std::pow(x, 3)
                    + A1.at<double>(0,4)*std::pow(x, 4)
                    + A1.at<double>(0,5)*std::pow(x, 5);
        s1+=abs(y1);
    }
    for(int i = 0; i<c.size();i+=5)
    {
        int y =c[i].y;
        double x = A2.at<double>(0,0) + A2.at<double>(0,1) * y +
                   A2.at<double>(0,2)*std::pow(y, 2) + A2.at<double>(0,3)*std::pow(y, 3) +
                   A2.at<double>(0,4)*std::pow(y, 5) + A2.at<double>(0,4)*std::pow(y, 5);
        s2+=abs(x);
    }
    std::cout<<"s1"<<s1<<std::endl;
    std::cout<<"s2"<<s2<<std::endl;
    lines.A1 = A1;
    lines.A2 = A2;
    if(s1 < s2)
    {
        //X,同时查找端点并返回

        lines.line_type = 1;
    } else{
        lines.line_type = 0;
    }
    return 1;
}