//
// Created by lx on 19-8-11.
//
#include "detect_line_space.h"

void detect_s_w_line::show()
{
    if (now_info.is_has_line)
    {
        for (int i=0; i<now_info.lines.size(); i++)
        {
            cv::line(img_ps_bgr,now_info.lines[i].fit_line1.p_toushi[0], now_info.lines[i].fit_line1.p_toushi[1],
                     cv::Scalar(255,0,0), 2, 8, 0);
            cv::line(img_ps_bgr,now_info.lines[i].fit_line2.p_toushi[0], now_info.lines[i].fit_line2.p_toushi[1],
                     cv::Scalar(255,0,0), 2, 8, 0);

        }
    }
    cv::imshow("img_ps_mask", img_ps_mask);
    cv::imshow("img_ps_mask_ipm", img_ps_mask_ipm);
    cv::imshow("img_ps_bgr", img_ps_bgr);
    cv::imshow("img_ps_bgr_ipm", img_ps_bgr_ipm);
    cv::waitKey(10);
}


int detect_s_w_line::detect()
{
    printf("start detect s_w lines...");
    //滤波
    if (!contours_filter())
    {
        std::cout<<"无车位"<<std::endl;
        now_info.is_has_line = 0;
        return -1;
    }
    contours_cluster(contours_filtered, con_clusters);

    //凸轮廓查找
    std::vector<cv::Point> dst;
    for(int i=0; i<contours_filtered.size(); i++)
    {
        extrct_convex_points(contours_filtered[i], dst, 30);
        contours_filtered_con.push_back(dst);
        dst.clear();
    }

    //RANSAC拟合//ransac 拟合直线
    for(int i=0;i<con_clusters.size();i++) //每一类
    {
        s_w_line_info tmp; //可能会找到单白线

        std::vector<cv::Point > pts;
        for(int j=0; j< con_clusters[i].size(); j++) //第i类的第j个轮廓
        {
            std::vector<cv::Point> tc = contours_filtered_con[con_clusters[i][j]];
            for(int m=0; m<tc.size(); m++)
            {
                pts.push_back(tc[m]);
            }
        }

        bool isfound_line2, isfound_line1;
        aps::RansacLine2D ransac_lines1;
        ransac_lines1.setObservationSet(pts);
        ransac_lines1.setRequiredInliers(int(pts.size()/7));
        ransac_lines1.setIterations(10);
        ransac_lines1.setTreshold(4);
        double start = static_cast<double>(cvGetTickCount());
        isfound_line1 = ransac_lines1.computeModel();
        double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
        std::cout << "ransac_lines1所用时间为:" << time/1000<<"ms"<<std::endl;
        //aps::LineModel fit_line;
        super_rect rc;
        ransac_lines1.getBestModel(rc.fit_line1);
        tmp.fit_line1.p_toushi.push_back(ransac_lines1.p_min);
        tmp.fit_line1.p_toushi.push_back(ransac_lines1.p_max);
        std::cout<<"ransac_lines1.notConsensusSet"<<ransac_lines1.m_notConsensusSet.size()<<std::endl;
        if (ransac_lines1.m_notConsensusSet.size() > 6 )
        {
            aps::RansacLine2D ransac_lines2;
            ransac_lines2.setObservationSet(ransac_lines1.m_notConsensusSet);
            ransac_lines2.setRequiredInliers(int(pts.size()/7));
            ransac_lines2.setIterations(10);
            ransac_lines2.setTreshold(4);
            start = static_cast<double>(cvGetTickCount());
            isfound_line2 = ransac_lines2.computeModel();
            time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
            std::cout << "ransac_lines2所用时间为:" << time/1000<<"ms"<<std::endl;
            ransac_lines2.getBestModel(rc.fit_line2);
            tmp.fit_line2.p_toushi.push_back(ransac_lines2.p_min);
            tmp.fit_line2.p_toushi.push_back(ransac_lines2.p_max);
        }

        if (isfound_line1 && isfound_line2)
        {
            now_info.is_has_line = 1;
            now_info.lines.push_back(tmp);
        }
    }

    return 1;
}

//函数重复使用，待完善
int detect_s_w_line::contours_filter()
{
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_ps_mask, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE );
    printf("滤波前所有的轮廓 n=%d\n",(int)contours.size());
    for(int i=0;i<contours.size();i++) //对所有的轮廓
    {
        int s = cv::contourArea(contours[i]);
        int len = cv::arcLength(contours[i],1);
        if( s > 200)
        {
            contours_filtered.push_back(contours[i]);
            printf("s=%d\n",s);
            printf("len=%d\n",len);
        }
    }
    return contours_filtered.size();
}
