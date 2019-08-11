// 计算凸缺陷 convexityDefect
//

#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

//输入，输出分别为输入的轮廓(一个轮廓)，输出为凸轮廓点集(对应的一个凸轮廓点集)
int extrct_convex_points(std::vector<cv::Point> &src,std::vector<cv::Point> &dst, int min_t) //在轮廓点中，根据凸包分析，将凸部分点集提取
{
    //求凸包
    vector<int> hull; //保存凸包点的索引(针对轮廓点集)
    double start = static_cast<double>(cvGetTickCount());
    convexHull(src, hull, true);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout << "计算凸包:" << time/1000<<"ms"<<endl;

    //判断是否有凸缺陷
    // 如果有凸缺陷就把它画出来
    if( isContourConvex(src) ) //判断轮廓points_01是否为凸
    {
        cout<<"src的轮廓是凸包"<<endl;
        dst = src;
        return 0;
    }
    else
    {
        cout<<"src的轮廓含有凹缺陷"<<endl;
        vector<Vec4i> defects;
        //求凹缺陷
        start = static_cast<double>(cvGetTickCount());
        convexityDefects(src, Mat(hull), defects);
        time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
        cout << "计算凸缺陷:" << time/1000<<"ms"<<endl;

        std::vector<int> points_dev_index; //凹点索引
        bool has_inv_convex = 0;// 标志 是否有满足条件的凹缺陷
        for(int i=0;i < defects.size();++i) //遍历所有的凹缺陷
        {
            std::cout<<"凹陷的距离="<<defects[i][3]/256 << std::endl;
            if(defects[i][3]/256 > min_t)
            {
                has_inv_convex = 1;
                for(int k=defects[i][0]; k<defects[i][1];k++)
                {
                    points_dev_index.push_back(k);
                }
            }
        }

        if (has_inv_convex)
        {
            //src轮廓索引 - 凹点集索引 = 凸点集索引
            std::vector<cv::Point> points_convex; //凸点集合
            int start_dex = 0;
            cout<<"contours[0].size()"<<src.size()<<endl;
            for(int m=0;m<src.size();m++)
            {
                if(m == points_dev_index[start_dex])
                {
                    start_dex++;
                }
                else
                {
                    points_convex.push_back(src[m]);
                }
            }
            dst = points_convex;
            return 1;  //含有满足条件的凹缺陷
        }
        else
        {
            dst = src;
            return 0;
        }
    }
}

double calu_dis_2point(cv::Point &p1, cv::Point &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
double calu_contours_dis(std::vector<cv::Point> &src1, std::vector<cv::Point> &src2)
{
    double now_dis;
    double min_dis = 1e9;
    for(int i=0;i<src1.size();i=i+5)
    {
        for(int j=0;j<src2.size();j=j+5)
        {
            now_dis = calu_dis_2point(src1[i],src2[j]);
            if(now_dis < min_dis)
            {
                min_dis = now_dis;
            }
        }
    }
    return min_dis;
}

bool is_in_vector(int a,std::vector<int> v)
{
    for(int i=0;i<v.size();i++)
    {
        if(a == v[i])
        {
            return 1;
        }
    }
    return 0;
}

//轮廓距离：最近点间的欧式距离
//输入 图片中含有的总轮廓个数
//输出类别-每个类别包含的轮廓索引。
int contours_cluster(std::vector<std::vector<cv::Point>> &src,
                     std::vector<std::vector<int>> & dst)
{
    const int num = src.size();
    double score[num][num] = {0,}; //二部匹配矩阵
    for(int h=0;h<num;h++)
    {
        for(int w=h+1;w<num;w++)
        {
            score[h][w] = calu_contours_dis(src[h],src[w]);
        }
    }

    //二部计算结果
    std::vector<std::vector<int>> match_s1; //stage one match result
    std::vector<bool> isCluster(src.size(),0); //h方向是否已经分配

    for(int h=0; h<num; h++) //第i行
    {
        std::vector<int> ct;
        ct.push_back(h);
        for(int w=h+1; w<num; w++)
        {
            if(score[h][w] < 20)
            {
                ct.push_back(w);
            }
        }
        match_s1.push_back(ct); //一类
    }

//    for(int i=0; i<match_s1.size();i++)
//    {
//        //std::cout<<"match_s1[i]="<<std::endl;
//        for(int j=0;j<match_s1[i].size();j++)
//        {
//            std::cout<<match_s1[i][j]<<" ";
//        }
//        std::cout<<endl;
//    }

    std::vector<std::vector<int>> cluser_result; //最终聚类结果
    bool now_c = 1;
    for(int i=0; i<match_s1.size();i++)
    {
        if (i == 0)
        {
            cluser_result.push_back(match_s1[i]);
        }
        else
        {
            for(int j=0; j<cluser_result.size(); j++)
            {
                if(is_in_vector(match_s1[i][0],cluser_result[j]))
                {
                    if(match_s1[i].size() > 1)
                    {
                        for(int k=1; k<match_s1[i].size(); k++)
                        {
                            cluser_result[j].push_back(match_s1[i][k]);
                        }
                    }
                    now_c = 0;
                }
            }
            if (now_c)
            {
                cluser_result.push_back(match_s1[i]);
            }
            else
            {
                now_c = 1;
            }
        }
        std::cout<<endl;
    }

//    for(int i=0; i<cluser_result.size();i++)
//    {
//        //std::cout<<"cluser_result[i]="<<std::endl;
//        for(int j=0;j<cluser_result[i].size();j++)
//        {
//            std::cout<<cluser_result[i][j]<<" ";
//        }
//        std::cout<<endl;
//    }
    dst = cluser_result;
    return 1;
}






int main()
{
    Mat im,im_rgb;
    im = imread("/home/lx/data/7.png",cv::IMREAD_GRAYSCALE);
    im_rgb = imread("/home/lx/data/7_2.png");
    resize(im, im,cv::Size(im.size()*3));
    resize(im_rgb, im_rgb,cv::Size(im_rgb.size()*3));

    bitwise_not(im,im);
    vector<vector<Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    double start = static_cast<double>(cvGetTickCount());
    findContours(im, contours, hierarchy, cv::RETR_CCOMP,
                 cv::CHAIN_APPROX_NONE);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout << "查找轮廓:" << time/1000<<"ms"<<endl;

    for(int i=0; i<contours.size(); i++)
    {
        for(int j=0; j< contours[i].size(); j++)
        {
            circle(im_rgb, contours[i][j], 1, Scalar(0,0,255), CV_FILLED, CV_AA);
        }
    }

   vector<Point> dst;
   extrct_convex_points(contours[0],dst,20);

    double min_dis;
    start = static_cast<double>(cvGetTickCount());
    min_dis = calu_contours_dis(contours[0], contours[1]);
    time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout << "计算轮廓间的距离:" << time/1000<<"ms"<<endl;
    cout<<"轮廓间的距离为" << min_dis <<endl;

    //轮廓聚类
    std::vector<std::vector<int >> con_clusters;
    start = static_cast<double>(cvGetTickCount());
    contours_cluster(contours,con_clusters);
    time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    cout << "轮廓聚类:" << time/1000<<"ms"<<endl;

    for(int i=0;i<con_clusters.size();i++) //每一类
    {
        cv::Scalar color(rand()&255, rand()&255,rand()&255);
        for(int j=0; j< con_clusters[i].size(); j++)
        {
            std::vector<Point> tc = contours[con_clusters[i][j]];
            for(int m=0; m<tc.size(); m++)
            {
                circle(im_rgb, tc[m], 1, color, CV_FILLED, CV_AA);
            }
        }
        imshow("im_rgb", im_rgb);
        waitKey(0);
    }

    imshow("im_rgb", im_rgb);
    waitKey(0);

    return 0;
}


//#include<opencv2/opencv.hpp>
//#include <stdio.h>
//#include <dirent.h>
//#include <string>
//#include <iostream>
//#include "math.h"
////全局cap
//std::vector<cv::Scalar> cmap;
//bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A);
//bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id);
//int pro_one_img(cv::Mat & label_png, cv::Mat & img_jpg);
//double calu_dis(cv::Point &p1,cv::Point &p2 );
//double calu_min_dis_countours(std::vector<cv::Point> &c1,std::vector<cv::Point> & c2);
//int cluster_con_min_dis(std::vector<std::vector<cv::Point>> &contours,
//                        std::vector<int> & classCluster); //根据最小距离约束对轮廓进行聚类
//
//int main()
//{
//    for(int i=0;i<100;i++)
//    {
//        cmap.push_back(cv::Scalar(rand()&255, rand()&255,rand()&255));
//    }
//
//    struct dirent *dirp;
//    std::string label_pred_dir = "/home/lx/data/surround_line8_test/left_demo_pred/";
//    std::string img_dir = "/home/lx/data/surround_line8_test/left_distort/";
//    std::string label_png_dir;
//    std::string img_jpg_dir;
//    cv::Mat label_png,img_jpg;
//
//    DIR* dir = opendir(label_pred_dir.c_str());
//    while ((dirp = readdir(dir)) != NULL)
//    {
//      if (dirp->d_type == DT_REG)
//      {
//        std::string tname = dirp->d_name;
//        label_png_dir = label_pred_dir + tname;
//        int numof = tname.find_first_of(".");
//        img_jpg_dir = img_dir + tname.substr(0,numof)+".jpg";
//        std::cout<<img_jpg_dir << std::endl;
//
//        label_png = cv::imread(label_png_dir, cv::IMREAD_GRAYSCALE);
//        img_jpg = cv::imread(img_jpg_dir);
//        pro_one_img(label_png,img_jpg);
//      }
//    }
//
//   /*
//  cv::Mat line_img = cv::Mat::zeros(label_png.rows, label_png.cols, CV_8UC1);
//  cv::Mat img_erode;
//
//
//  cv::imshow("label_png", label_png);
//
//  get_mask_img(label_png, line_img, 1);
//  //dilate
//  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//  cv::erode(line_img,img_erode, element);
//
//  //find contours
//  std::vector<std::vector<cv::Point>> contours;
//  //contours[0] is the first contours
//  std::vector<cv::Vec4i> hierarchy;
//  cv::findContours(img_erode, contours, hierarchy, cv::RETR_CCOMP,
//    cv::CHAIN_APPROX_SIMPLE);
//  int ca = cv::contourArea(contours[0]); //find contours
//  //int cl = cv::arcLength(contours[0]); //calu contours length
//
//  cv::RotateRect minRect = cv::minAreaRect(contours[0]);
//  cv::Point2f vertex[4];  minRect.points(vertex);
//  for (int i=0; i<4; i++)
//  {
//
//  }
//
//  std::cout<< "ca" << ca << std::endl;
//
//
//
//  int index = 0;
//  for(; index >= 0; index = hierarchy[index][0])
//  {
//    cv::Scalar color(rand()&255, rand()&255,rand()&255);
//
//    cv::drawContours(img_jpg, contours, index, color,
//      2,8,hierarchy);
//  }
//
//
//
//
//
//  cv::imshow("line_img", line_img);
//  cv::imshow("img_erode", img_erode);
//  cv::imshow("img_jpg", img_jpg);
//  cv::waitKey(0);
//
//
//
//
//  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
//  image.setTo(cv::Scalar(100, 0, 0));
//  std::vector<cv::Point> points;
//  points = contours[0];
//  for (int i = 0; i < points.size(); i++)
//  {
//    cv::circle(image, points[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
//  }
//
//  //cv::polylines(image, points, false, cv::Scalar(0, 255, 0), 1, 8, 0); //line point to point
//
//  cv::Mat A;
// //y = a0 + a1*x + a2*x2 + a3*x3
//  polynomial_curve_fit(points, 3, A);
//  std::cout << "A = " << A << std::endl;
//
//
//  std::vector<cv::Point> points_fitted;
//  for (int x = 0; x < 640; x++)
//  {
//      double y = A.at<double>(0, 0) + A.at<double>(1, 0) * x +
//      A.at<double>(2, 0)*std::pow(x, 2) + A.at<double>(3, 0)*std::pow(x, 3);
//      points_fitted.push_back(cv::Point(x, y));
//  }
//  cv::polylines(image, points_fitted, false, cv::Scalar(0, 255, 255), 1, 8, 0);
//
//  cv::imshow("image", image);
//  cv::waitKey(0);
//  */
//  return 0;
//}
//
//bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
//{
//    //Number of key points
//    int N = key_point.size();
//    //构造矩阵X
//    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
//    for (int i = 0; i < n + 1; i++)
//    {
//        for (int j = 0; j < n + 1; j++)
//        {
//            for (int k = 0; k < N; k++)
//            {
//                X.at<double>(i, j) = X.at<double>(i, j) +
//                                     std::pow(key_point[k].x, i + j);
//            }
//        }
//    }
//
//    //构造矩阵Y
//    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
//    for (int i = 0; i < n + 1; i++)
//    {
//        for (int k = 0; k < N; k++)
//        {
//            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
//                                 std::pow(key_point[k].x, i) * key_point[k].y;
//        }
//    }
//
//    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
//    //求解矩阵A
//    cv::solve(X, Y, A, cv::DECOMP_LU);
//    return true;
//}
//
//bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id)
//{
//    for(int h=0;h<src.rows; ++h)
//    {
//        for(int w=0;w<src.cols; ++w)
//        {
//            if (src.at<uchar>(h,w) == id )
//            {
//                dst.at<uchar>(h,w) = 255;
//            }
//        }
//    }
//    return true;
//}
//int pro_one_img(cv::Mat & label_png, cv::Mat & img_jpg)
//{
//    // get line img
//    cv::Mat img_con = img_jpg.clone();
//    cv::Mat pred_line = cv::Mat::zeros(label_png.rows, label_png.cols, CV_8UC1);
//    cv::Mat img_erode, img_dilate;
//    get_mask_img(label_png,pred_line, 1);
//    // erode
//    cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
//    cv::Mat element_i = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//    cv::erode(pred_line,img_erode, element_erode);
//    cv::dilate(img_erode, img_dilate,element_i);
//
//    // 1.0 find contours
//    std::vector<std::vector<cv::Point>> contours; //contours[0] is the first contours
//    std::vector<cv::Vec4i> hierarchy;
//    cv::Mat timg = img_dilate .clone();
//    cv::findContours(timg , contours, hierarchy, cv::RETR_CCOMP,
//                     cv::CHAIN_APPROX_SIMPLE);
//    // 1.1 draw
//    int index = 0;
//    for(; index >= 0; index = hierarchy[index][0])
//    {
//        cv::Scalar color(rand()&255, rand()&255,rand()&255);
//        cv::drawContours(img_con, contours, index, color,
//                         2,8,hierarchy);
//    }
//
//    //轮廓==1时，通过面积约束过滤噪音，直接拟合。
//    //轮廓>1时，聚类得到新的contours<vector<cv::Point> >  cluster_con;
//
//
//
//    std::vector<bool> isCluster(contours.size(),0); //指示该轮廓是否已分配类别
//    std::vector<int> classCluster(contours.size(),0); //指示该轮廓所属的类别
//    //对于属于[0]的类别
//    int nowClasses = 0;
//    int dex_now = 0; //第0个索引还没分配类别。
//    for (int j=0; j<contours.size(); j++)
//    {
//        if (isCluster[j] == 0) //该位置还没分类类别
//        {
//            dex_now = j;
//            for(int i=0; i<contours.size();i++)
//            {
//                if (isCluster[i] == 0) //没有分配类别
//                {
//                    double mins = calu_min_dis_countours(contours[dex_now],contours[i]);
//                    if (mins < 80)
//                    {
//                        isCluster[i] = 1; //分配类别
//                        classCluster[i] = nowClasses;
//                    }
//                }
//            }
//            isCluster[dex_now] = 1; //分配类别
//            classCluster[dex_now] = nowClasses;
//            nowClasses++;
//        }
//    }
//
//    for (int i=0;i<classCluster.size();i++)
//    {
//        std::cout << "classCluster:"<<classCluster[i] << std::endl;
//    }
//
//
//    for(int i=0; i<contours.size();++i)
//    {
//        for(int j=0; j< contours[i].size();j++)
//        {
//            cv::circle(img_jpg,contours[i][j],3,cmap[classCluster[i]],1,8,0);
//        }
//    }
//
//
//    cv::imshow("label_png", label_png);
//    cv::imshow("pred_line", pred_line);
//    cv::imshow("img_erode", img_erode);
//    cv::imshow("img_dilate", img_dilate);
//    cv::imshow("img_jpg", img_jpg);
//    cv::imshow("img_con", img_con);
//    cv::waitKey(0);
//    return 1;
//}
//
////计算两个轮廓间最小距离
//double calu_min_dis_countours(std::vector<cv::Point> &c1,std::vector<cv::Point> & c2)
//{
//    int min_c1=641,max_c1=-1,min_c2=641,max_c2=-1;
//    int dex_min_c1 = 0,dex_max_c1=0,dex_min_c2=0,dex_max_c2=0;
//    for(int i=0;i<c1.size();i++)
//    {
//        if(c1[i].x < min_c1)
//        {
//            min_c1 = c1[i].x;
//            dex_min_c1 = i;
//        }
//        if(c1[i].x > max_c1)
//        {
//            max_c1 = c1[i].x;
//            dex_max_c1 = i;
//        }
//    }
//    for(int i=0;i<c2.size();i++)
//    {
//        if(c2[i].x < min_c2)
//        {
//            min_c2 = c2[i].x;
//            dex_min_c2 = i;
//        }
//        if(c2[i].x > max_c2)
//        {
//            max_c2 = c2[i].x;
//            dex_max_c2 = i;
//        }
//    }
//    double pp[4];
//    pp[0] = calu_dis(c1[dex_min_c1],c2[dex_min_c2]);
//    pp[1] = calu_dis(c1[dex_min_c1],c2[dex_max_c2]);
//    pp[2] = calu_dis(c1[dex_max_c1],c2[dex_min_c2]);
//    pp[3] = calu_dis(c1[dex_max_c1],c2[dex_max_c2]);
//
//    std::cout<<min_c1<<" "<<max_c1<<" "<<min_c2<<" "<<max_c2<<std::endl;
//    std::cout<<c1[dex_min_c1]<<" "<<c1[dex_max_c1]<<" "<<c2[dex_min_c2]<<" "<<c2[dex_max_c2]<<std::endl;
//    std::cout<<pp[0]<<" "<<pp[1]<<" "<<pp[2]<<" "<<pp[3]<<std::endl;
//    double mins=10000.0;
//    for(int i=0;i<4;i++)
//    {
//        if(pp[i] < mins)
//        {
//            mins = pp[i];
//        }
//    }
//    return mins;
//}
//
//double calu_dis(cv::Point &p1,cv::Point &p2 )
//{
//    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
//}
//
////根据最小距离约束对轮廓进行聚类
////输入:轮廓vector
////输出:同轮廓等长的vector,元素值表示
//
//int cluster_con_min_dis(std::vector<std::vector<cv::Point>> &contours,
//                        std::vector<int> & classCluster)
//{
//
//    return 1;
//}
