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
#include <fstream>

//C code
struct PSP_Vec4f
{
    float x1;
    float y1;
    float x2;
    float y2;
};
struct PSP_Vec4d
{
    double x1;
    double y1;
    double x2;
    double y2;
};
struct PSP_Vec4i
{
    int x1;
    int y1;
    int x2;
    int y2;
};
struct PSP_LineSet_d
{
    static int num; //the number of lines number
    PSP_Vec4d lines[];
};


//C++
const  double pi = 3.141592653;
struct s_w_line_info
{
    int line_type; //X型=1， Y=0
    aps::LineModel fit_line1; //RANSAC拟合的线
    aps::LineModel fit_line2; //RANSAC拟合的线


    int minp; //某个轴上的最小值
    int maxp;
    cv::Point left;
    cv::Point right;
    std::vector<cv::Point> point_fit; //拟合使用的点
    cv::Mat A1; //多项式系数
    cv::Mat A2;
};
struct line_now_info
{
    bool is_has_line; //是否含有s_w
    std::vector<s_w_line_info> lines;
};


//一个潜在的车位引导线
//用三条线段表示(边缘直线，中心直线的斜率方程及端点坐标)
//
struct super_rect
{
    aps::LineModel fit_line1; //RANSAC拟合的线
    aps::LineModel fit_line2; //RANSAC拟合的线
    aps::LineModel fit_line_mid; //两条拟合线的角平分线
    double alpha_line12; //line 1 和 line2之间的夹角


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
    super_rect sr1; //两个super_rect
    super_rect sr2;

    std::vector<cv::Point> pts; //四个顶点坐标(IPM坐标系) -> 最终输出
};
struct  ps_one_frame
{
    bool is_has_ps;
    std::vector<parking_space_info> ps;
};

struct parking_space_line_type
{
    cv::Vec6f pos_line;
    cv::Vec6f neg_line;
    cv::Vec6f mid_line;
};

//自定义L角点结构体。
struct L_shape_type
{
    int line_type;   //分割线的类型。1-垂直；2-54;3-138度
    int point_type;  //角点的类型(pos L-points, neg L-point)
    double alpha; //分割线与停止线的夹角
    parking_space_line_type stop_line; //一条停止线
    cv::Vec6f vertical_line; //与停止线垂直的直线
    cv::Point2f point_L; //L角点坐标
    cv::Point2f point_end; //L角点 沿着引导线 终端坐标
};

struct canditate_L_shape_type
{
    float e; //置信度
    cv::Vec6f line1;
    cv::Vec6f line2;
    cv::Point2f pt;  //交点
};



struct  parking_space_type
{
    double e; //置信度
    int tracker_id; //考虑帧间跟踪，可能会用到

    L_shape_type left_point; //两个入口角点确定一个车位
    L_shape_type right_point;

};






struct  now_ps_type
{
    bool is_has_ps;
    std::vector<parking_space_type> ps;
};




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



int solve_mid_line(aps::LineModel &l1, aps::LineModel &l2, aps::LineModel &l_mid);




//common API
//迭代求解方程 theta_d = theta ( 1 + D1*theta^2 + D2*theta^4 + D3*theta^6 + D4*theta^8 )
double solve_theta_d(double theta_d,cv::Mat & D);
//
//输入参数： 图像点的坐标 imagePoints，世界坐标系下点的z轴坐标 Z，世界坐标系转换到相机坐标系的 旋转矩阵 rvec，平移向量tvec，相机内参矩阵K，相机畸变系数D
//输出参数： 世界坐标系下点的坐标 objectPoints
void computeWorldcoordWithZ (std::vector<cv::Point3f>& objectPoints, std::vector<cv::Point2f>& imagePoints, std::vector<double>& Z,cv::Mat & rvec,
                             cv::Mat & tvec, cv::Mat & K, cv::Mat & D);

//根据相机参数计算IPM H 及 H_inv
bool img_undistort2topview(cv::Mat & inter_params,
                           std::vector<double> distortion,
                           cv::Mat & rvecs,
                           cv::Mat & tvecs,
                           std::string direction,
                           std::string camera_type,
                           cv::Mat & img_undistorted,
                           cv::Mat & img_ipm);

//鱼眼图像->去畸变图像
bool undistort_fish_img(cv::Mat & inter_params,
                        std::vector<double> distortion,
                        double rx,
                        double ry,
                        cv::Size imageSize,
                        cv::Mat & fish_img,
                        cv::Mat & dst_img);

//计算两个车位之间的距离(数据关联用)
//目前版本: dis = 两个入口角点的局部坐标L2距离平均值
bool calu_dis_ps(cv::Vec4f & line1, cv::Vec4f & line2);


bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id);

//calu dis in 2 lines
//Vec4f = x0,y0,x1,y1. (x0,y0) is the left, bottom(x0=x1) point
double calu_dis_2lines(cv::Vec4f & line1, cv::Vec4f & line2);
double calu_dis_2lines_m2(cv::Vec4f & line1, cv::Vec4f & line2);
double calu_dis_2lines_m2(cv::Vec6f & line1, cv::Vec6f & line2);

double calu_dis_2point(cv::Point &p1, cv::Point &p2);
double calu_dis_2point2f(cv::Point2f &p1, cv::Point2f &p2);
//计算直线
cv::Point2f calu_intersection_point_2lines(cv::Vec6f & line1, cv::Vec6f & line2);
cv::Point2f calu_intersection_point_2lines(cv::Vec4f & line1, cv::Vec4f & line2);
cv::Point2f calu_point_2lines(cv::Vec6f & line1, cv::Vec6f & line2);


//计算点到直线的距离
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);



//rank=1  y = b + k * x
//rank=2  y = x0 + x1 * x + x2 * x^2
bool polynomial_curve_fit(std::vector<cv::Point>& points, int rank, cv::Mat& coef);

//对同一条直线上的线段族 LSM拟合
//输入，线段族。输入 一条线段
bool lines_lsm_fit(std::vector<cv::Vec4f> & lines, cv::Vec6f & line, cv::Mat & img_pos_s_lines);

bool find_end_point(std::vector<cv::Vec6f> & pos_separating_lines,
                    cv::Mat & img_ps_mask_ipm);

//求线段与x轴正方向的夹角 is_ = 1时为弧度值, 否则为角度值
//范围 [0-180)
double calu_alpha_line(cv::Vec4f & line, bool is_);
double calu_alpha_line(cv::Vec6f & line, bool is_);
//计算两条线段间的平行度
double calu_para_2lines(cv::Vec4f & line1, cv::Vec4f & line2);
double calu_para_2lines(cv::Vec6f & line1, cv::Vec6f & line2);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
* input
* std::vector<cv::Vec4f> & plines 透视图中hough直线段
* std::vector<cv::Vec4f> & plines_ipm 俯视图中hough直线段
* cv::Mat & img_pred 俯视图分割结果 , 二值化图
* output
* std::vector<cv::Vec4f> & pos_lines 正线段集合
* * * * * * * * * * * * * * * * * * * * * * * * * * */
bool classify_hough_lines(std::vector<cv::Vec4f> & plines,
                          std::vector<cv::Vec4f> & plines_ipm,
                            cv::Mat & img_pred,
                            int search_len,
                            double black_t, //0值比例阈值
                            std::vector<cv::Vec4f> & pos_lines,
                            std::vector<cv::Vec4f> & neg_lines);

//根据直线在透视图的关系判断正负性
bool classify_hough_lines_p(std::vector<cv::Vec4f> & plines,
                          std::vector<cv::Vec4f> & plines_ipm,
                          cv::Mat & img_pred,
                          int search_len,
                          double black_t, //0值比例阈值
                          std::vector<cv::Vec4f> & pos_lines,
                          std::vector<cv::Vec4f> & neg_lines);
/*
查找潜在的车位停止线
input: pos_separating_lines, neg_separating_lines
output:  int,是否查找到潜在的车位停止线, 车位停止线条数
车位停止线信息 std::vector<parking_space_line_type> stop_line
find stop lines accoding to 1.line-k and 2.the dis from pos-line to pos-line(6-20cm)
 */
bool find_stop_lines(std::vector<cv::Vec6f> & pos_separating_lines,
                     std::vector<cv::Vec6f> & neg_separating_lines,
                     int & stop_line_num,
                     std::vector<parking_space_line_type> & stop_line);




//Hough 后直线初次combined
bool lines_combined(std::vector<cv::Vec4f> & plines, std::vector<cv::Vec4f> & plines_combined);

bool combined_line_cluster(std::vector<cv::Vec4f> & plines_combined, std::vector<std::vector<cv::Vec4f>> &separating_lines);

bool points_topview_2_perspective(cv::Vec4f & points_src, cv::Vec4f & points_dst, cv::Mat H_inv); //逐点从俯视图转为透视图

//在入口角点已知的正负分割线间聚类车位
//输入分别为pos, neg车位分割线
//输出为当前帧车位信息
//std::vector<cv::Vec6f> pos_separating_lines,分别为left下,left上点坐标,和k,b值
bool find_parking_space_in_posneg_separating_lines(
        std::vector<cv::Vec6f> & pos_separating_lines,
        std::vector<cv::Vec6f> & neg_separating_lines,
        now_ps_type & ps_all);

/*
 * input: pos and neg separating_lines set(after fitting for every lines)
 * input: line_alpha_error
 * output: L shape point type vector.
 * */
bool find_Lshape_point(std::vector<cv::Vec6f> & pos_separating_lines,
                       std::vector<cv::Vec6f> & neg_separating_lines,
                       std::vector<parking_space_line_type> & stop_line,
                       double line_alpha_error,
                       std::vector<L_shape_type> & L_shapes);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * std::string 分割
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
std::vector<std::string> string_split(const std::string &s, const std::string &seperator);

/*
 * 在分隔线集合中确定L角点
 * input:
 * */
bool find_Lshape_point(std::vector<cv::Vec6f> & pos_separating_lines,
                       std::vector<cv::Vec6f> & neg_separating_lines,
                       std::vector<parking_space_line_type> & stop_line,
                       double line_alpha_error,
                       std::vector<L_shape_type> & L_shapes);

 /* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * if not fine stop line,
 * find entrance points according to line end point (search)
 * input: pos_separating_lines, pos separating_lines set
 * input: neg_separating_lines set
 * input: img_ps_mask_ipm, 分割二值图
 * input: img_ps_bgr_ipm, 原始图IPM
 * output: L_shapes 集合
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
bool find_Lshape_point(std::vector<cv::Vec6f> & pos_separating_lines,
                       std::vector<cv::Vec6f> & neg_separating_lines,
                       cv::Mat & img_ps_mask_ipm,
                       cv::Mat & img_ps_bgr_ipm,
                       std::vector<L_shape_type> & L_shapes);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 在L角点集中查找车位
 * 输入： std::vector<L_shape_type> L_shapes, 当前帧的所有角点
 * 输出： now_ps_type ps_all, 当前帧的所有车位信息
 * 返回值：标志位
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
bool find_ps_in_L_shape_points(std::vector<L_shape_type> & L_shapes,
                               now_ps_type & ps_all);


/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * find_hough_lines_set from image, in perspective view.
 * input：   img_ps_mask  (cv::Mat 0,255)
 * output：  hough_lines   Vec4f[0],[1] is the left-bottom points.x,y
 *           in perspective view.
 * return：
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
int find_hough_lines_set(cv::Mat & img_ps_mask,
                         std::vector<cv::Vec4f> & hough_lines);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * find_hough_lines_set from image
 * input：   img_ps_mask  (cv::Mat 0,255)
 * input:    ipm_mat perspective view -> bird-top view
 * output：  in bird-top view hough_lines
 *           Vec4f[0],[1] is the left-bottom points.x,y
 * return：
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
int find_hough_lines_set(cv::Mat & img_ps_mask,
                         cv::Mat & ipm_mat,
                         std::vector<cv::Vec4f> & hough_lines);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 点集/线段集 透视/逆透视变换
 * 同时对逆透视变换后的点。重新sort-> 第一个点为左点(或者下, 当x1=x2时)
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
int  perspective_transform_points(std::vector<cv::Point> &src,
                                  std::vector<cv::Point> &dst,
                                   cv::Mat H);
int  perspective_transform_lines(std::vector<cv::Vec4f> &src,
                                 std::vector<cv::Vec4f> &dst,
                                 cv::Mat H);
int  perspective_transform_lines(std::vector<cv::Vec6f> &src,
                                 std::vector<cv::Vec6f> &dst,
                                 cv::Mat H);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 根据采样距离, 线段, 方向标志位, 求采样举行的顶点
 * input： type 1-left； 2--right， 3--top， 4--bottom
 * return：
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
bool get_rect_points(cv::Vec4f & line,
                     cv::Mat & im,
                     int d,
                     int type,
                     std::vector<cv::Point> & pts);
bool get_rect_points(cv::Vec6f & line,
                     cv::Mat & im,
                     int d,
                     int type,
                     std::vector<cv::Point> & pts);


/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 根据采样距离, 已知线段, 目标point， 方向标志位, 求采样线段上255点所占的比例
 *                  |
 * -----------------|----
 *          0，0，0，|，0，255，0
 * input： type 1-top； 2--bottom， 3--left， 4--right
 * return：
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
double calu_255r_in_v_line(cv::Vec6f & line,
                     cv::Mat & im,
                     cv::Point2f &pt,
                     int d,
                     int type);
double calu_255r_in_v_line(cv::Vec4f & line,
                           cv::Mat & im,
                           cv::Point2f &pt,
                           int d,
                           int type);
/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 根据采样矩形的角点, 求 二值化图内对应点 255 的个数 占比
 * input： type 1-透视图， 2-俯视图
 * return：
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
double calu_points_num_inRect(std::vector<cv::Point> & pts,
                     cv::Mat & im, int type);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 对拟后的长线段寻找分割线端点
 * input
 * neg_separating_lines 待处理的分割线
 * img_ps_mask_ipm 二值俯视图
 * img_ps_bgr_ipm BGR俯视图 用于确定分割线的完整性
 * output
 * pos_separating_lines_dst 输出的包含终端信息的分割线
 * L_point 以角点的形式输出
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
int find_separating_line_end_point(std::vector<cv::Vec6f> & pos_separating_lines,
                   cv::Mat & img_ps_mask_ipm,
                   cv::Mat & img_ps_bgr_ipm,
                   std::vector<cv::Vec6f> & pos_separating_lines_dst,
                   std::vector<L_shape_type> & L_point);

/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 对候选车位进一步确定
 * 输入，某一个候选车位，mask ipm， 返回值是否为车位，及是车位的概率
 * 返回值 ：-1 不是车位； >0 是车位，且值代表车位的置信度
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
double classify_h_parking_space(parking_space_type & tps,
                                cv::Mat & mask);


#endif //TARGET_RECOGNITION_TOOLS_H























