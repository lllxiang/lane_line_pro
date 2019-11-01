//
// Created by lx on 19-8-11.
//
#ifndef TARGET_RECOGNITION_DETECT_PARKING_SPACE_H
#define TARGET_RECOGNITION_DETECT_PARKING_SPACE_H

#include "tools.h"

class parking_space
{
public:
     parking_space();
     ~parking_space();
    void show();
    void show_ipm();
    int detect_ps();
    int detect_ps_ipm();
    int detect_closeed_ps();
    bool save_ps_to_txt_file(int frame_num, now_ps_type & ps_info, std::string file_name); //保存检测结果至txt文件

    cv::Mat H;
    cv::Mat H_inv;

    cv::Mat img_ps_mask;        //0=bg, 255=车位引导线, ps=parking space..透视图
    cv::Mat img_ps_mask_ipm;    //同上，俯视图
    cv::Mat img_ps_bgr; //透视图，原始图片
    cv::Mat img_ps_bgr_ipm; //俯视图，原始图片


    std::vector<cv::Vec4f> hough_lines;
    std::vector<cv::Vec4f> hough_lines_ipm;

    //plines[0],  cv::Vec4f[0][1] is left,bottom points;;; [2][3] is right or top points
    std::vector<cv::Vec4f> pos_lines;   //正负线段集合
    std::vector<cv::Vec4f> neg_lines;

    std::vector<std::vector<cv::Vec4f>> pos_lines_cluster; //正负线段聚类结果
    std::vector<std::vector<cv::Vec4f>> neg_lines_cluster;

    std::vector<cv::Vec6f> pos_separating_lines; // 正负分割线 lsm拟合结果
    std::vector<cv::Vec6f> neg_separating_lines;

    std::vector<cv::Vec6f>  pos_separating_lines_dst;
    std::vector<cv::Vec6f>  neg_separating_lines_dst;

    std::vector<parking_space_line_type> parking_space_lines;

    std::vector<L_shape_type> left_L_point;
    std::vector<L_shape_type> right_L_point;

    now_ps_type  ps_h; //车位候选集
    now_ps_type ps_all;



    std::string perspectiveFileName_left;
    std::vector<cv::Vec4f> plines_combined;          //
    std::vector<std::vector<cv::Vec4f>> separating_lines; //separating_lines


    std::vector<std::vector<cv::Point>> contours_filtered; //滤波后
    std::vector<std::vector<cv::Point>> contours_filtered_con; //凸轮廓

    std::vector<std::vector<int>>  con_clusters; //聚类结果


    std::vector<super_rect> rects; //所有检测出的超矩形

    ps_one_frame now_info; //当前帧的所有车位线信息

    cv::VideoWriter outputVideo;
    std::vector<cv::Scalar> cmap;
    std::string file_dir;
    int frame_num_now;
    int now_frame_count;


};










#endif //TARGET_RECOGNITION_DETECT_PARKING_SPACE_H
