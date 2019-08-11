//
// Created by lx on 19-8-11.
//
#ifndef TARGET_RECOGNITION_DETECT_PARKING_SPACE_H
#define TARGET_RECOGNITION_DETECT_PARKING_SPACE_H

#include "tools.h"

class parking_space
{
public:
    //parking_space();

    void show(); //可视化

    int contours_filter(); //滤波,透视图中，将小于一定值的轮廓去掉

    int detect();


    cv::Mat img_ps_mask;  //0=bg, 255=车位引导线, ps=parking space..透视图
    cv::Mat img_ps_mask_ipm; //同上，俯视图
    cv::Mat img_ps_bgr; //透视图，原始图片
    std::vector<std::vector<cv::Point>> contours; //原始轮廓

    std::vector<std::vector<cv::Point>> contours_filtered; //滤波后
    std::vector<std::vector<cv::Point>> contours_filtered_con; //凸轮廓

    std::vector<std::vector<int>>  con_clusters; //聚类结果


    ps_one_frame ps_now;
    std::vector<super_rect> rects; //所有检测出的超矩形

};










#endif //TARGET_RECOGNITION_DETECT_PARKING_SPACE_H
