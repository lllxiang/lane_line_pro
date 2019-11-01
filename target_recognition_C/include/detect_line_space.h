//
// Created by lx on 19-8-11.
//

#ifndef TARGET_RECOGNITION_DETECT_LINE_SPACE_H
#define TARGET_RECOGNITION_DETECT_LINE_SPACE_H

#include "tools.h"

class detect_s_w_line
{
public:
    //detect_s_w_line();

    int detect(); //detect and return the number of s_w lines
    void show();
    int  contours_filter();


    //member value
    std::vector<std::vector<cv::Point>> contours; //原始轮廓

    std::vector<std::vector<cv::Point>> contours_filtered; //滤波后
    std::vector<std::vector<cv::Point>> contours_filtered_con; //凸轮廓

    std::vector<std::vector<int>>  con_clusters; //聚类结果

    line_now_info now_info;
    cv::Mat img_ps_mask;  //0=bg, 255=s_w, 透视图
    cv::Mat img_ps_mask_ipm; //同上，俯视图
    cv::Mat img_ps_bgr; //透视图，原始图片
    cv::Mat img_ps_bgr_ipm; //俯视图，原始图片


};


#endif //TARGET_RECOGNITION_DETECT_LINE_SPACE_H
