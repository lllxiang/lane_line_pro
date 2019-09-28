//
// Created by lx on 19-9-4.
//

#ifndef TARGET_RECOGNITION_KALMAN_TRACKER_H
#define TARGET_RECOGNITION_KALMAN_TRACKER_H

#include "tools.h"

//计算两个车位的匹配距离
//车位对应角点的欧式距离平均值
double calu_parking_space_dis(parking_space_type & ps_detect,
    parking_space_type & ps_tracker)
{
    return (calu_dis_2lines_m2(ps_tracker.line_left, ps_detect.line_left)
            + calu_dis_2lines_m2(ps_tracker.line_right, ps_detect.line_right)) /2.0f;
}

//检测响应和跟踪响应的匹配函数
bool associate_detections_to_trackers(now_ps_type & detections,
                                      now_ps_type & trackers,
                                      double threadhold)
{
    //返回值：
    std::vector<std::vector<int>> matched; //检测ID-跟踪ID
    std::vector<int> unmatched_detections;
    std::vector<int> unmatched_trackers;

    if (trackers.ps.size() == 0)
    {
        //左边界, 无轨迹, 只有unmatched_detections
        for(int i=0; i<detections.ps.size(); i++)
        {
            unmatched_detections.push_back(i);
        }
    }

    if (detections.ps.size() == 0)
    {
        //右边界, 场景结束。只有unmatched_trackers
        for (int i = 0; i < trackers.ps.size() ; ++i)
        {
            unmatched_trackers.push_back(i);
        }
    }

    //求匹配矩阵
    cv::Mat match_mat = cv::Mat::zeros(cv::Size(trackers.ps.size(), detections.ps.size()), CV_32FC1);
    for(int d=0; d<detections.ps.size();d++)
    {

    }
    return true;
}

class kalman_tracker
{

};

#endif //TARGET_RECOGNITION_KALMAN_TRACKER_H
