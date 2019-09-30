//
// Created by lx on 19-8-11.
//

#include "detect_parking_space.h"
#include <chrono>

void parking_space::show2()
{
    cv::Mat img_hough_lines_ipm = img_ps_bgr_ipm.clone();
    std::cout << "ipm img size= " << img_hough_lines_ipm.cols << " " << img_hough_lines_ipm.rows << std::endl;
    cv::Mat combined_lines_cluster = img_ps_bgr_ipm.clone();

    cv::Mat img_ps_mask_ipm3c;
    img_ps_mask_ipm3c = cv::Mat::zeros(img_ps_mask_ipm.size(), CV_8UC3);
    for(int h=0;h<img_ps_mask_ipm.rows; ++h)
    {
        for(int w=0;w<img_ps_mask_ipm.cols; ++w)
        {
            for(int c=0; c<3; c++)
            {
                img_ps_mask_ipm3c.at<cv::Vec3b>(h,w)[c] = img_ps_mask_ipm.at<uchar>(h,w);
            }
        }
    }
    cv::Mat img_pos_neg_result = img_ps_mask_ipm3c.clone();
    cv::Mat img_pos_neg_cluster = img_ps_mask_ipm3c.clone();
    cv::Mat img_lines_fitting = img_ps_mask_ipm3c.clone();
    cv::Mat img_lines_fitting_end = img_ps_mask_ipm3c.clone();
    cv::Mat img_L_points = img_ps_mask_ipm3c.clone();
    cv::Mat img_ps_h = img_ps_bgr_ipm.clone();

    //1. 透视图中显示hough
    for (int i=0; i<hough_lines.size();i++)
    {
        cv::Vec4f point1 = hough_lines[i];
        line(img_ps_bgr, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cmap[i], 3, cv::LINE_AA);
    }
    //1. IPM图中显示线段
    for (int i=0; i<hough_lines_ipm.size();i++)
    {
        cv::Vec4f point1 = hough_lines_ipm[i];
        line(img_hough_lines_ipm, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cmap[i], 3, cv::LINE_AA);
        cv::circle(img_hough_lines_ipm, cv::Point(point1[0], point1[1]),4,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_hough_lines_ipm, cv::Point(point1[2], point1[3]),4,cv::Scalar(0,255,0),-1,8,0); //G
    }

    //2. 显示正负线段检测结果
    for(int i=0; i< pos_lines.size(); i++)
    {
        cv::Vec4f point1 = pos_lines[i];
        line(img_pos_neg_result, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(img_pos_neg_result, cv::Point(point1[0], point1[1]),4,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_pos_neg_result, cv::Point(point1[2], point1[3]),4,cv::Scalar(0,255,0),-1,8,0); //G
    }
    for(int i=0; i< neg_lines.size(); i++)
    {
        cv::Vec4f point1 = neg_lines[i];
        line(img_pos_neg_result, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        cv::circle(img_pos_neg_result, cv::Point(point1[0], point1[1]),4,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_pos_neg_result, cv::Point(point1[2], point1[3]),4,cv::Scalar(0,255,0),-1,8,0); //G
    }

    //3.0 显示正负线段聚类结果
    for(int i=0; i< pos_lines_cluster.size(); i++)
    {
        for(int j=0; j< pos_lines_cluster[i].size(); j++)
        {
            cv::Vec4f point1 = pos_lines_cluster[i][j];
            line(img_pos_neg_cluster, cv::Point(point1[0], point1[1]),
                 cv::Point(point1[2], point1[3]), cmap[i], 2, cv::LINE_AA);
        }
    }

    for(int i=0; i< neg_lines_cluster.size(); i++)
    {
        for(int j=0; j< neg_lines_cluster[i].size(); j++)
        {
            cv::Vec4f point1 = neg_lines_cluster[i][j];
            line(img_pos_neg_cluster, cv::Point(point1[0], point1[1]),
                 cv::Point(point1[2], point1[3]), cmap[10+i], 2, cv::LINE_AA);
        }
    }

    //4.0 显示正负分割线拟合结果 (以长线段表示)
    for(int i=0; i<pos_separating_lines.size();i++)
    {
        cv::Vec6f point1 = pos_separating_lines[i];

        line(img_lines_fitting, cv::Point(point1[2], point1[3]),
             cv::Point(point1[0], point1[1]), cv::Scalar(0,0,255), 2, cv::LINE_AA);
        //cv::circle(img_lines_fitting, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0);
        //cv::circle(img_lines_fitting, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,255,0),-1,8,0);
    }
    for(int i=0; i<neg_separating_lines.size();i++)
    {
        cv::Vec6f point1 = neg_separating_lines[i];
        line(img_lines_fitting, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cv::Scalar(255,0,0), 2, cv::LINE_AA);
        //cv::circle(img_lines_fitting, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0);
        //cv::circle(img_lines_fitting, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,255,0),-1,8,0);
    }

    //显示正负分割线 (含有端点)
    for(int i=0; i<pos_separating_lines_dst.size();i++)
    {
        cv::Vec6f point1 = pos_separating_lines_dst[i];
        line(img_lines_fitting_end, cv::Point(point1[2], point1[3]),
             cv::Point(point1[0], point1[1]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
        cv::circle(img_lines_fitting_end, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0);
        cv::circle(img_lines_fitting_end, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,255,0),-1,8,0);
    }
    for(int i=0; i<neg_separating_lines_dst.size();i++)
    {
        cv::Vec6f point1 = neg_separating_lines_dst[i];
        line(img_lines_fitting_end, cv::Point(point1[2], point1[3]),
             cv::Point(point1[0], point1[1]), cv::Scalar(255,0,0), 1, cv::LINE_AA);
        cv::circle(img_lines_fitting_end, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0);
        cv::circle(img_lines_fitting_end, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,255,0),-1,8,0);
    }


    //显示L角点
    if(left_L_point.size() > 0)
    {
        for (int i = 0; i < left_L_point.size() ; ++i)
        {
            cv::circle(img_L_points, left_L_point[i].point_L,1,cv::Scalar(0,0,255),-1,8,0);
        }
    }
    if(right_L_point.size() > 0)
    {
        for (int i = 0; i < right_L_point.size() ; ++i)
        {
            cv::circle(img_L_points, right_L_point[i].point_L,1,cv::Scalar(255,0,0),-1,8,0);
        }
    }
    //显示候选车位集 及 真实车位集
    if (ps_h.is_has_ps > 0)
    {
        for (int i = 0; i < ps_h.ps.size(); ++i)
        {
            line(img_ps_h, ps_h.ps[i].left_point.point_L,
                 ps_h.ps[i].right_point.point_L, cv::Scalar(255,255,0), 1, cv::LINE_AA);
            line(img_ps_h, ps_h.ps[i].left_point.point_end,
                 ps_h.ps[i].right_point.point_end, cv::Scalar(255,255,0), 1, cv::LINE_AA);

            if (ps_h.ps[i].e > 0)
            {
                line(img_ps_h, ps_h.ps[i].left_point.point_L,
                     ps_h.ps[i].right_point.point_L, cv::Scalar(255,0,0), 1, cv::LINE_AA);
                line(img_ps_h, ps_h.ps[i].left_point.point_end,
                     ps_h.ps[i].right_point.point_end, cv::Scalar(255,0,0), 1, cv::LINE_AA);
            }
        }
    }


    cv::imshow("img_ps_mask", img_ps_mask);
    cv::imshow("img_ps_bgr", img_ps_bgr);
    cv::imshow("img_hough_lines_ipm", img_hough_lines_ipm);
    cv::imshow("img_pos_neg_result", img_pos_neg_result);
    cv::imshow("img_pos_neg_cluster", img_pos_neg_cluster);
    cv::imshow("img_lines_fitting", img_lines_fitting);
    cv::imshow("img_lines_fitting_end", img_lines_fitting_end);
    cv::imshow("img_L_points", img_L_points);
    cv::imshow("img_ps_h", img_ps_h);

//  cv::imshow("img_ps_mask", img_ps_mask);
    cv::imshow("img_find_Lshape_point", img_ps_bgr_ipm);
//    cv::imshow("img_3c_hough", img_ps_mask_ipm3c);
//
//
//
//
//    cv::imshow("img_ps_mask_ipm", img_ps_mask_ipm);

    cv::waitKey(1);

}



int parking_space::detect_test()
{
//    int roi_x = 320;
//    int roi_y = 0;
//    int roi_w = 220;
//    int roi_h = 298;
//    std::cout << "cols" << img_ps_bgr_ipm.cols << std::endl;
//    std::cout << "rows" << img_ps_bgr_ipm.rows << std::endl;
//    //cv::resize(img_ps_mask_ipm,img_ps_mask_ipm,cv::Size(img_ps_mask_ipm.cols/2,img_ps_mask_ipm.rows));
//
//    //find contours
//    std::vector<cv::Vec4i> hierarchy;
//    double start = static_cast<double>(cvGetTickCount());
//    cv::Mat timg = img_ps_mask_ipm.clone();
//
//
//    cv::findContours(timg(cv::Rect(roi_x,roi_y,roi_w,roi_h)), contours, hierarchy, cv::RETR_EXTERNAL,
//                     cv::CHAIN_APPROX_NONE );
//
//    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
//    std::cout << "1->findContours及滤波耗时:" << time/1000<<"ms"<<std::endl;
//
//    cv::Mat img_t = cv::Mat::zeros(img_ps_mask_ipm.size(), CV_8UC1);
//    int hough_points = 0;
//    for (int i=0;i<contours.size();i++)
//    {
//        for(int j=0; j<contours[i].size();j++)
//        {
//            img_t.at<uchar>(contours[i][j].y, contours[i][j].x + roi_x) = 255;
//            hough_points++;
//        }
//    }
//    std::cout<<"hough_points"<<hough_points << std::endl;
//    //hough find all lines
//    start = static_cast<double>(cvGetTickCount());
//    cv::HoughLinesP(img_t(cv::Rect(roi_x,roi_y,roi_w,roi_h)), plines, 1, CV_PI / 180, 30, 30,6);
//    time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
//    std::cout << "1->hough边缘检测耗时:" << time/1000<<"ms"<<std::endl;
//
//    for(int i=0; i< plines.size(); i++)
//    {
//        plines[i][0] = plines[i][0] + roi_x;
//        plines[i][2] = plines[i][2] + roi_x;
//    }

    // 1.0
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    src.push_back(cv::Point2f(190,0));
    src.push_back(cv::Point2f(549,0));
    src.push_back(cv::Point2f(549,277));
    src.push_back(cv::Point(190,277));
    cv::perspectiveTransform(src, dst, H_inv);
    cv::Point verticesp[1][4];
    verticesp[0][0] = dst[0];
    verticesp[0][1] = dst[1];
    verticesp[0][2] = dst[2];
    verticesp[0][3] = dst[3];
    const cv::Point *ppt[1] = {verticesp[0]};
    int npt[] = {4};
    cv::Mat mask;
    mask = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1); //750, 298
    fillPoly(mask, ppt, npt, 1, 255);
    cv::bitwise_and(mask, img_ps_mask, mask);
    find_hough_lines_set(mask, hough_lines);
    double start;

    perspective_transform_lines(hough_lines, hough_lines_ipm, H);

    //2.0 判断正负线段
    pos_lines.clear();
    neg_lines.clear();
    std::vector<cv::Vec4f>  tpos_lines;
    std::vector<cv::Vec4f>  tneg_lines;

    start = static_cast<double>(cvGetTickCount());
    classify_hough_lines(hough_lines,
                         hough_lines_ipm,
                         img_ps_mask_ipm,
                         20, 0.8,tpos_lines, tneg_lines);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "2->classify_hough_lines:" << time/1000<<"ms"<<std::endl;

    perspective_transform_lines(tpos_lines, pos_lines, H);
    perspective_transform_lines(tneg_lines, neg_lines, H);

    //3.0 对正负线段分别聚类
    pos_lines_cluster.clear();
    neg_lines_cluster.clear();
    combined_line_cluster(pos_lines, pos_lines_cluster);
    combined_line_cluster(neg_lines, neg_lines_cluster);
    std::cout <<"pos_lines_cluster.size()"<<pos_lines_cluster.size()<<std::endl;

    //4.0 对正负线段箤拟合(LSM)
    pos_separating_lines.clear();
    neg_separating_lines.clear();
    for (int i = 0; i < pos_lines_cluster.size(); ++i)
    {
        cv::Vec6f line;
        lines_lsm_fit(pos_lines_cluster[i], line, img_ps_bgr_ipm);
        pos_separating_lines.push_back(line);
    }
    for (int i = 0; i < neg_lines_cluster.size(); ++i)
    {
        cv::Vec6f line;
        lines_lsm_fit(neg_lines_cluster[i], line, img_ps_bgr_ipm);
        neg_separating_lines.push_back(line);
    }


    //1.0 处理没有停止线的情况
    //依据端点确定 角点
    find_separating_line_end_point(neg_separating_lines,
                                   img_ps_mask_ipm,
                                   img_ps_bgr_ipm,
                                   neg_separating_lines_dst,
                                   left_L_point);

    find_separating_line_end_point(pos_separating_lines,
                                   img_ps_mask_ipm,
                                   img_ps_bgr_ipm,
                                   pos_separating_lines_dst,
                                   right_L_point);
    bool debug = 1;
    std::cout << "pos_separating_lines_dst" << pos_separating_lines_dst.size() << std::endl;
    //在角点集合中确定车位: 角点->先验约束(距离,平行度)得到车位候选集->车位验证-> 符合的车位关联后融合输出

    ps_h.ps.clear();
    if (left_L_point.size() == 0 || right_L_point.size() == 0)
    {//无车位
        ps_h.is_has_ps = 0;
        ps_h.ps.clear();
    } else
    {
        for (int i = 0; i < left_L_point.size(); ++i)
        {
            for (int j = 0; j < right_L_point.size(); ++j)
            {
                std::cout << "left_L_point[i].point_L " << left_L_point[i].point_L << std::endl;
                std::cout << "right_L_point[i].point_L" << right_L_point[j].point_L << std::endl;
                if (right_L_point[j].point_L.x < left_L_point[i].point_L.x)
                {
                    continue;
                }
                double param = fabsf(left_L_point[i].alpha - right_L_point[j].alpha);

                double dis = calu_dis_2lines_m2(left_L_point[i].vertical_line,right_L_point[j].vertical_line);
                std::cout << "param = " << param << std::endl;
                std::cout << "dis = " << dis << std::endl;
                std::cout << "left_L_point[i].alpha = " << left_L_point[i].alpha << std::endl;
                std::cout << "right_L_point[j].alpha = " << right_L_point[j].alpha << std::endl;

                if (left_L_point[i].alpha < 70)
                {
                    std::cout << "left_L_point[i].vertical_line[0] = " << left_L_point[i].vertical_line[0] << std::endl;
                    std::cout << "left_L_point[i].vertical_line[1] = " << left_L_point[i].vertical_line[1] << std::endl;
                    std::cout << "left_L_point[i].vertical_line[2] = " << left_L_point[i].vertical_line[2] << std::endl;
                    std::cout << "left_L_point[i].vertical_line[3] = " << left_L_point[i].vertical_line[3] << std::endl;

                }


                if (param < 5 && dis > 90 && dis < 130) //垂直车位
                {
                    parking_space_type pst;
                    pst.left_point = left_L_point[i];
                    pst.right_point = right_L_point[j];
                    pst.e = param;
                    ps_h.is_has_ps = 1;
                    ps_h.ps.push_back(pst);

                }
            }
        }
    }
    //至此得到所有的候选车位, 车位确定
    for (int k = 0; k < ps_h.ps.size(); ++k)
    {
        double is_ps = classify_h_parking_space(ps_h.ps[k],
                                                img_ps_mask_ipm);
        if (is_ps > 0)
        {
            ps_h.ps[k].e = is_ps;
        }
        std::cout << "----------------is_ps" << is_ps << std::endl;
    }












//    //查找停止线
//    std::vector<parking_space_line_type> stop_line;
//    int stop_line_num=0;
//    find_stop_lines(     pos_separating_lines,
//                         neg_separating_lines,
//                         stop_line_num,
//                         stop_line);
//    if (stop_line_num < 1) //搜索端点确定入口角点
//    {
//        find_Lshape_point(pos_separating_lines,
//                          neg_separating_lines,
//                          img_ps_mask_ipm,
//                          img_ps_bgr_ipm,
//                          L_shapes);
//    }
//    else
//    {
//        //根据相交条件确定入口角点。
//        find_Lshape_point(pos_separating_lines,
//                          neg_separating_lines,
//                          stop_line,
//                          3, L_shapes);
//    }
    //至此得到所有角点信息。在角点中查找车位
    // find_ps_in_L_shape_points(L_shapes, ps_all);
    now_frame_count++;
    return 0;
}

parking_space::parking_space()
{
    for(int i=0;i<30;i++)
    {
        cmap.push_back(cv::Scalar(rand()%255, rand()%255, rand()%255));
    }

    cv::Size sWH = cv::Size(640,480);
    std::string outputVideoPath = "../test.avi";
    outputVideo.open(outputVideoPath, CV_FOURCC('M', 'P', '4', '2'), 20.0, sWH);

    H = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
    H_inv = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
    const std::string perspectiveFileName_left = "/home/lx/data/suround_view_src_data/calibration/weishi/fish4/perspectiveTransformCalibration.txt";
    readPerspectiveParams(perspectiveFileName_left, H, H_inv);

}
parking_space::~parking_space()
{


}



















