//
// Created by lx on 19-8-11.
//

#ifndef TARGET_RECOGNITION_TOOLS_H
#define TARGET_RECOGNITION_TOOLS_H

#include<opencv2/opencv.hpp>
#include <iostream>

#include "ReadParams.h"
#include <dirent.h>
#include <ransac_line2d.h>


struct line_info
{
    int line_type; //X型=1， Y=0
    int minp; //某个轴上的最小值
    int maxp;
    cv::Point left;
    cv::Point right;
    std::vector<cv::Point> point_fit; //拟合使用的点
    cv::Mat A1; //多项式系数
    cv::Mat A2;
};
struct super_rect //
{
    aps::LineModel fit_line1; //RANSAC拟合的线
    aps::LineModel fit_line2; //RANSAC拟合的线

    cv::RotatedRect baseRect; //在baseRect的基础上增加若干属性
    float l_edge;  //矩形框的长边 长度
    float s_edge;  //矩形框的短边 长度
    std::vector<cv::Point> l_edge_point; //长边的两个端点
    std::vector<cv::Point> s_edge_point; //长边的两个端点
    float l_edge_angle; //长轴相对于x正方向的夹角
};
struct  parking_space_info
{
    double e; //置信度
    int id; //考虑帧间跟踪，可能会用到
    super_rect sr1; //车位引导线
    super_rect sr2; //车位引导线2
    std::vector<cv::Point> pts; //四个顶点坐标(IPM坐标系) -> 最终输出
};
struct  ps_one_frame
{
    bool is_has_ps;
    std::vector<parking_space_info> ps;
};

//计算两点间距离
double calu_dis_2point(cv::Point &p1, cv::Point &p2);

//计算点到直线的距离
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);

//计算两个轮廓间距离(最小)
double calu_contours_dis(std::vector<cv::Point> &src1, std::vector<cv::Point> &src2);

//判断点是否在vector内
bool is_in_vector(int a,std::vector<int> v);

// 轮廓集合聚类，根据轮廓间的欧氏距离
// 轮廓距离：最近点间的欧式距离
//输入 图片中含有的总轮廓个数
//输出类别-每个类别包含的轮廓索引。
int contours_cluster(std::vector<std::vector<cv::Point>> &src,
                     std::vector<std::vector<int>> & dst);

// 轮廓凸点集提取
// 输入，输出分别为输入的轮廓(一个轮廓)，输出为凸轮廓点集(对应的一个凸轮廓点集)
//在轮廓点中，根据凸包分析，将凸部分点集提取
int extrct_convex_points(std::vector<cv::Point> &src,std::vector<cv::Point> &dst, int min_t);


#endif //TARGET_RECOGNITION_TOOLS_H
