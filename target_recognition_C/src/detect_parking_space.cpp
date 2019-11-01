//
// Created by lx on 19-8-11.
//

#include "detect_parking_space.h"
#include <chrono>
void parking_space::show_ipm() {
    //1.1 俯视图中显示hough直线段 及端点
    cv::Mat img_hough_lines_ipm = img_ps_bgr_ipm.clone();
    std::cout << "ipm img size= " << img_hough_lines_ipm.cols << " " << img_hough_lines_ipm.rows << std::endl;
    for (int i=0; i<hough_lines_ipm.size();i++)
    {
        cv::Vec4f point1 = hough_lines_ipm[i];
        line(img_hough_lines_ipm, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cmap[i], 2, cv::LINE_AA);
        cv::circle(img_hough_lines_ipm, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_hough_lines_ipm, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
    }


    //1.1
    cv::imshow("1.1->Hough检测直线段(俯视图)", img_hough_lines_ipm);
    cv::waitKey(0);

}
void parking_space::show()
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
    cv::Mat img_hough_lines;  cv::cvtColor(img_ps_mask,img_hough_lines,CV_GRAY2BGR);

    cv::Mat img_pos_neg_result = img_ps_mask_ipm3c.clone();
    cv::Mat img_pos_neg_cluster = img_ps_mask_ipm3c.clone();
    cv::Mat img_lines_fitting = img_ps_mask_ipm3c.clone();
    cv::Mat img_lines_fitting_end = img_ps_mask_ipm3c.clone();
    cv::Mat img_L_points = img_ps_mask_ipm3c.clone();
    cv::Mat img_ps_h = img_ps_bgr_ipm.clone();

    //1.0 透视图中显示hough直线段,端点
    for (int i=0; i<hough_lines.size();i++)
    {
        cv::Vec4f point1 = hough_lines[i];
        line(img_hough_lines, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cmap[i], 2, cv::LINE_AA);
        cv::circle(img_hough_lines, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_hough_lines, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
    }
    //1.1 俯视图中显示hough直线段 及端点
    for (int i=0; i<hough_lines_ipm.size();i++)
    {
        cv::Vec4f point1 = hough_lines_ipm[i];
        line(img_hough_lines_ipm, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cmap[i], 2, cv::LINE_AA);
        cv::circle(img_hough_lines_ipm, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_hough_lines_ipm, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
    }

    //2.0 显示正负线段检测结果
    for(int i=0; i< pos_lines.size(); i++)
    {
        cv::Vec4f point1 = pos_lines[i];
        line(img_pos_neg_result, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(img_pos_neg_result, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_pos_neg_result, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
    }
    for(int i=0; i< neg_lines.size(); i++)
    {
        cv::Vec4f point1 = neg_lines[i];
        line(img_pos_neg_result, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        cv::circle(img_pos_neg_result, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
        cv::circle(img_pos_neg_result, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
    }

    //3.0 显示正负线段聚类结果
    for(int i=0; i< pos_lines_cluster.size(); i++)
    {
        for(int j=0; j< pos_lines_cluster[i].size(); j++)
        {
            cv::Vec4f point1 = pos_lines_cluster[i][j];
            line(img_pos_neg_cluster, cv::Point(point1[0], point1[1]),
                 cv::Point(point1[2], point1[3]), cmap[i], 2, cv::LINE_AA);
            cv::circle(img_pos_neg_cluster, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
            cv::circle(img_pos_neg_cluster, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
        }
    }

    for(int i=0; i< neg_lines_cluster.size(); i++)
    {
        for(int j=0; j< neg_lines_cluster[i].size(); j++)
        {
            cv::Vec4f point1 = neg_lines_cluster[i][j];
            line(img_pos_neg_cluster, cv::Point(point1[0], point1[1]),
                 cv::Point(point1[2], point1[3]), cmap[10+i], 2, cv::LINE_AA);
            cv::circle(img_pos_neg_cluster, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0); //B
            cv::circle(img_pos_neg_cluster, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0); //R
        }
    }

    //4.0 显示正负分割线拟合结果 (以长线段表示)
    for(int i=0; i<pos_separating_lines.size();i++)
    {
        cv::Vec6f point1 = pos_separating_lines[i];

        line(img_lines_fitting, cv::Point(point1[2], point1[3]),
             cv::Point(point1[0], point1[1]), cv::Scalar(0,0,255), 2, cv::LINE_AA);
        cv::circle(img_lines_fitting, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0);
        cv::circle(img_lines_fitting, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0);
    }
    for(int i=0; i<neg_separating_lines.size();i++)
    {
        cv::Vec6f point1 = neg_separating_lines[i];
        line(img_lines_fitting, cv::Point(point1[0], point1[1]),
             cv::Point(point1[2], point1[3]), cv::Scalar(255,0,0), 2, cv::LINE_AA);
        cv::circle(img_lines_fitting, cv::Point(point1[0], point1[1]),3,cv::Scalar(255,0,0),-1,8,0);
        cv::circle(img_lines_fitting, cv::Point(point1[2], point1[3]),3,cv::Scalar(0,0,255),-1,8,0);
    }

    //4.1 显示正负分割线, 及分隔线端点(已排除不完整的分割线)
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

    //5.0 显示L角点
    if(left_L_point.size() > 0)
    {

        for (int i = 0; i < left_L_point.size() ; ++i)
        {
            cv::Vec6f vline = left_L_point[i].vertical_line;
            line(img_L_points, cv::Point(vline[2], vline[3]),
                 cv::Point(vline[0], vline[1]), cv::Scalar(0,0,255), 2, cv::LINE_AA);

            cv::circle(img_L_points, left_L_point[i].point_L,3,cv::Scalar(0,0,255),-1,8,0);//R-left point
        }
    }
    if(right_L_point.size() > 0)
    {
        for (int i = 0; i < right_L_point.size() ; ++i)
        {
            cv::Vec6f vline = right_L_point[i].vertical_line;
            line(img_L_points, cv::Point(vline[2], vline[3]),
                 cv::Point(vline[0], vline[1]), cv::Scalar(255,0,0), 2, cv::LINE_AA);
            cv::circle(img_L_points, right_L_point[i].point_L,3,cv::Scalar(255,0,0),-1,8,0);//B-right point
        }
    }
    //6.0 显示候选车位集合
    if (ps_h.is_has_ps > 0)
    {
        for (int i = 0; i < ps_h.ps.size(); ++i)
        {
            line(img_ps_h, ps_h.ps[i].left_point.point_L,
                 ps_h.ps[i].right_point.point_L, cv::Scalar(255,255,0), 1, cv::LINE_AA);
            line(img_ps_h, ps_h.ps[i].left_point.point_end,
                 ps_h.ps[i].right_point.point_end, cv::Scalar(255,255,0), 1, cv::LINE_AA);
        }
    }

//显示候选车位集 及 真实车位集
//    if (ps_h.is_has_ps > 0)
//    {
//        for (int i = 0; i < ps_h.ps.size(); ++i)
//        {
//            line(img_ps_h, ps_h.ps[i].left_point.point_L,
//                 ps_h.ps[i].right_point.point_L, cv::Scalar(255,255,0), 1, cv::LINE_AA);
//            line(img_ps_h, ps_h.ps[i].left_point.point_end,
//                 ps_h.ps[i].right_point.point_end, cv::Scalar(255,255,0), 1, cv::LINE_AA);
//
//            if (ps_h.ps[i].e > 0)
//            {
//                line(img_ps_h, ps_h.ps[i].left_point.point_L,
//                     ps_h.ps[i].right_point.point_L, cv::Scalar(255,0,0), 1, cv::LINE_AA);
//                line(img_ps_h, ps_h.ps[i].left_point.point_end,
//                     ps_h.ps[i].right_point.point_end, cv::Scalar(255,0,0), 1, cv::LINE_AA);
//            }
//        }
//    }


    //1.0
    cv::imshow("1.0->Hough检测直线段(透视图)", img_hough_lines);
    //1.1
    cv::imshow("1.1->Hough检测直线段(俯视图)", img_hough_lines_ipm);

    //2.0
    cv::imshow("2.0->正负线段分类结果", img_pos_neg_result);
    //3.0
    cv::imshow("3.0->正负线段分别聚类结果", img_pos_neg_cluster);
    //4.0
    cv::imshow("4.0->正负分割线同类拟合结果", img_lines_fitting);
    cv::imshow("4.1->正负分割线同类拟合结果(终点)", img_lines_fitting_end);
    //5.0
    cv::imshow("5.0->L角点", img_L_points);
    //6.0
    cv::imshow("6.0->候选车位", img_ps_h);

//  cv::imshow("img_ps_mask", img_ps_mask);
//    cv::imshow("img_find_Lshape_point", img_ps_bgr_ipm);
//    cv::imshow("img_3c_hough", img_ps_mask_ipm3c);
//
//
//
//
//    cv::imshow("img_ps_mask_ipm", img_ps_mask_ipm);

    cv::waitKey(0);

}

int parking_space::detect_closeed_ps()
{
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
    find_hough_lines_set(img_ps_mask, img_ps_bgr, hough_lines);
    double start;

    perspective_transform_lines(hough_lines, hough_lines_ipm, H);

    pos_lines_cluster.clear();
    combined_line_cluster(hough_lines_ipm , pos_lines_cluster);

    pos_separating_lines.clear();
    for (int i = 0; i < pos_lines_cluster.size(); ++i)
    {
        cv::Vec6f line;
        lines_lsm_fit(pos_lines_cluster[i], line, img_ps_bgr_ipm);
        pos_separating_lines.push_back(line);
    }
    //求所有的候选L型角点(交点, 且满足夹角约束90度)
    std::vector<canditate_L_shape_type> c_Lshapes; //所有的候选角点
    for (int i = 0; i < pos_separating_lines.size(); ++i)
    {
        for (int j = i; j < pos_separating_lines.size(); ++j)
        {
            cv::Vec6f line1 = pos_separating_lines[i];
            cv::Vec6f line2 = pos_separating_lines[j];
            double alpha = calu_para_2lines(line1, line2);
            if (abs(alpha- 90) < 10) //垂直的L角点
            {
                //double
                canditate_L_shape_type clt;
                std::cout<<"alpha="<<alpha<<std::endl;
                clt.pt = calu_intersection_point_2lines(line1,line2);
                double alpha_line1 = calu_alpha_line(line1, 0); //line1与x轴正方向的夹角
                double alpha_line2 = calu_alpha_line(line2, 0);
                //限制车位位置
                if(abs(alpha_line1 - 90) < 20 && (abs(alpha_line2 - 0) < 20
                                                 || abs(alpha_line2 - 180) < 20)) //alpha_line1为分割线
                {
                    clt.line1 = line1; //分割线
                    clt.line2 = line2; //停止线
                    c_Lshapes.push_back(clt);
                }
                else if (abs(alpha_line2 - 90) < 20 && (abs(alpha_line1 - 0) < 20
                                                        || abs(alpha_line1 - 180) < 20))
                {
                    clt.line1 = line2; //分割线
                    clt.line2 = line1; //停止线
                    c_Lshapes.push_back(clt);
                }

            }
        }
    }

    //L角点验证
    cv::Mat img_cut_point = img_ps_mask_ipm.clone();
    cv::cvtColor(img_cut_point,img_cut_point,CV_GRAY2BGR);

    for (int i = 0; i < c_Lshapes.size(); ++i)
    {
        cv::Vec6f line_p = c_Lshapes[i].line2;
        cv::Vec6f line_v = c_Lshapes[i].line1;

        cv::Point2f mid_p((line_p[0]+line_p[2])/2,(line_p[1]+line_p[3])/2);
        cv::Point2f mid_v((line_v[0]+line_v[2])/2,(line_v[1]+line_v[3])/2);

        double bottom255 = calu_255r_in_v_line(line_p,
                            img_ps_mask_ipm,
                            mid_p,
                            6,
                            2);
        double top255 = calu_255r_in_v_line(line_p,
                                               img_ps_mask_ipm,
                                               mid_p,
                                               6,
                                               1);
        double left255 = calu_255r_in_v_line(line_v,
                                             img_ps_mask_ipm,
                                             mid_p,
                                             6,
                                             3);
        double right255 = calu_255r_in_v_line(line_v,
                                             img_ps_mask_ipm,
                                             mid_p,
                                             6,
                                             4);

        std::cout<<"top255=" << top255 << std::endl;
        std::cout<<"bottom255=" << bottom255 << std::endl;

        if (bottom255 > 0.5 && top255 < 0.5
                               && left255 > 0.5 && right255 < 0.5) // && left255 > 0.7 && right255 < 0.4
        {
            cv::circle(img_cut_point,c_Lshapes[i].pt,1,cv::Scalar(0,0,255),-1,8,0);
            cv::line(img_cut_point,cv::Point(line_p[0],line_p[1]),cv::Point(line_p[2],line_p[3]),cv::Scalar(255,0,0),
                 1,8,0);
            cv::line(img_cut_point,cv::Point(line_v[0],line_v[1]),cv::Point(line_v[2],line_v[3]),cv::Scalar(0,255,0),
                  1,8,0);

            c_Lshapes[i].e = 1;
        }
    }        


    for (int k = 0; k < c_Lshapes.size(); ++k)
    {
        if(c_Lshapes[k].e == 1)
        {
            cv::circle(img_cut_point,c_Lshapes[k].pt,1,cv::Scalar(0,0,255),-1,8,0);
        }

    }




//    //2.0 判断正负线段
//    pos_lines.clear();
//    neg_lines.clear();
//    std::vector<cv::Vec4f>  tpos_lines;
//    std::vector<cv::Vec4f>  tneg_lines;
//
//    start = static_cast<double>(cvGetTickCount());
//    classify_hough_lines(hough_lines,
//                         hough_lines_ipm,
//                         img_ps_mask_ipm,
//                         20, 0.8,tpos_lines, tneg_lines);
//    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
//    std::cout << "2->classify_hough_lines:" << time/1000<<"ms"<<std::endl;
//
//    perspective_transform_lines(tpos_lines, pos_lines, H);
//    perspective_transform_lines(tneg_lines, neg_lines, H);
//
//    //3.0 对正负线段分别聚类
//    pos_lines_cluster.clear();
//    neg_lines_cluster.clear();
//    combined_line_cluster(pos_lines, pos_lines_cluster);
//    combined_line_cluster(neg_lines, neg_lines_cluster);
//    std::cout <<"pos_lines_cluster.size()"<<pos_lines_cluster.size()<<std::endl;
//
//    //4.0 对正负线段箤拟合(LSM)
//    pos_separating_lines.clear();
//    neg_separating_lines.clear();
//    for (int i = 0; i < pos_lines_cluster.size(); ++i)
//    {
//        cv::Vec6f line;
//        lines_lsm_fit(pos_lines_cluster[i], line, img_ps_bgr_ipm);
//        pos_separating_lines.push_back(line);
//    }
//    for (int i = 0; i < neg_lines_cluster.size(); ++i)
//    {
//        cv::Vec6f line;
//        lines_lsm_fit(neg_lines_cluster[i], line, img_ps_bgr_ipm);
//        neg_separating_lines.push_back(line);
//    }
//
//    //1.0 处理没有停止线的情况
//    //依据端点确定 角点
//    find_separating_line_end_point(neg_separating_lines,
//                                   img_ps_mask_ipm,
//                                   img_ps_bgr_ipm,
//                                   neg_separating_lines_dst,
//                                   left_L_point);
//
//    find_separating_line_end_point(pos_separating_lines,
//                                   img_ps_mask_ipm,
//                                   img_ps_bgr_ipm,
//                                   pos_separating_lines_dst,
//                                   right_L_point);
//    bool debug = 1;
//    std::cout << "pos_separating_lines_dst" << pos_separating_lines_dst.size() << std::endl;
//    //在角点集合中确定车位: 角点->先验约束(距离,平行度)得到车位候选集->车位验证-> 符合的车位关联后融合输出
//
//    ps_h.ps.clear();
//    if (left_L_point.size() == 0 || right_L_point.size() == 0)
//    {//无车位
//        ps_h.is_has_ps = 0;
//        ps_h.ps.clear();
//    } else
//    {
//        for (int i = 0; i < left_L_point.size(); ++i)
//        {
//            for (int j = 0; j < right_L_point.size(); ++j)
//            {
//                std::cout << "left_L_point[i].point_L " << left_L_point[i].point_L << std::endl;
//                std::cout << "right_L_point[i].point_L" << right_L_point[j].point_L << std::endl;
//                if (right_L_point[j].point_L.x < left_L_point[i].point_L.x)
//                {
//                    continue;
//                }
//                double param = fabsf(left_L_point[i].alpha - right_L_point[j].alpha);
//
//                double dis = calu_dis_2lines_m2(left_L_point[i].vertical_line,right_L_point[j].vertical_line);
//                std::cout << "param = " << param << std::endl;
//                std::cout << "dis = " << dis << std::endl;
//                std::cout << "left_L_point[i].alpha = " << left_L_point[i].alpha << std::endl;
//                std::cout << "right_L_point[j].alpha = " << right_L_point[j].alpha << std::endl;
//
//                if (left_L_point[i].alpha < 70)
//                {
//                    std::cout << "left_L_point[i].vertical_line[0] = " << left_L_point[i].vertical_line[0] << std::endl;
//                    std::cout << "left_L_point[i].vertical_line[1] = " << left_L_point[i].vertical_line[1] << std::endl;
//                    std::cout << "left_L_point[i].vertical_line[2] = " << left_L_point[i].vertical_line[2] << std::endl;
//                    std::cout << "left_L_point[i].vertical_line[3] = " << left_L_point[i].vertical_line[3] << std::endl;
//
//                }
//
//
//                if (param < 5 && dis > 90 && dis < 130) //垂直车位
//                {
//                    parking_space_type pst;
//                    pst.left_point = left_L_point[i];
//                    pst.right_point = right_L_point[j];
//                    pst.e = param;
//                    ps_h.is_has_ps = 1;
//                    ps_h.ps.push_back(pst);
//
//                }
//            }
//        }
//    }
//    //至此得到所有的候选车位, 车位确定
//    for (int k = 0; k < ps_h.ps.size(); ++k)
//    {
//        double is_ps = classify_h_parking_space(ps_h.ps[k],
//                                                img_ps_mask_ipm);
//        if (is_ps > 0)
//        {
//            ps_h.ps[k].e = is_ps;
//        }
//        std::cout << "----------------is_ps" << is_ps << std::endl;
//    }
//
//    //查找停止线
//    std::vector<parking_space_line_type> stop_line;
//    int stop_line_num;
//    find_stop_lines(     pos_separating_lines,
//                         neg_separating_lines,
//                         stop_line_num,
//                         stop_line);
//
//    std::cout<<"stop_line_num="<<stop_line_num<<std::endl;
    return 0;
}
int parking_space::detect_ps_ipm() {
    double start;
    start = static_cast<double>(cvGetTickCount());
    find_hough_lines_set(img_ps_mask_ipm, img_ps_bgr_ipm, hough_lines_ipm);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    //std::cout << "2->classify_hough_lines:" << time/1000<<"ms"<<std::endl;





    return 0;
}

int parking_space::detect_ps()
{
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
    //1.0 在透视图中进行hough变换
    find_hough_lines_set(img_ps_mask, img_ps_bgr, hough_lines);
    double start;
    //1.1 hough直线段 透视图->俯视图.对超出边界的直线段,进行边界处理
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
    perspective_transform_lines(tpos_lines, pos_lines, H);
    perspective_transform_lines(tneg_lines, neg_lines, H);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "2->classify_hough_lines:" << time/1000<<"ms"<<std::endl;

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

    //1.0 判断没有停止线后:
    //5.0 依据端点确定 角点
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

    //在角点集合中确定车位: 角点->先验约束(距离,平行度)得到车位候选集->车位验证-> 符合的车位关联后融合输出
    //输入: 左角点集合, 右角点集合
    //输出: ps_h 候选车位集合
    ps_h.ps.clear();
    if (left_L_point.size() == 0 || right_L_point.size() == 0)
    {
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
                if (param < 6 && dis > 90 && dis < 130) //1.8-2.6m 垂直车位
                {
                    parking_space_type pst;
                    pst.left_point = left_L_point[i];
                    pst.right_point = right_L_point[j];
                    pst.e = 1;
                    ps_h.is_has_ps = 1;
                    ps_h.ps.push_back(pst);
                }
                else if (param < 5 && dis > 150 && dis < 250) //3m-5m 平行车位
                {

                }
            }
        }
    }
    //std::string file_dir = "/home/lx/data/ceping_data/weishi/seq4/pred_txt/a.txt";
    save_ps_to_txt_file(frame_num_now,ps_h, file_dir);
    //至此得到所有的候选车位, 车位确定
//    for (int k = 0; k < ps_h.ps.size(); ++k)
//    {
//        double is_ps = classify_h_parking_space(ps_h.ps[k],
//                                                img_ps_mask_ipm);
//        if (is_ps > 0)
//        {
//            ps_h.ps[k].e = is_ps;
//        }
//        std::cout << "----------------is_ps" << is_ps << std::endl;
//    }

//    //查找停止线
//    std::vector<parking_space_line_type> stop_line;
//    int stop_line_num;
//    find_stop_lines(     pos_separating_lines,
//                         neg_separating_lines,
//                         stop_line_num,
//                         stop_line);
//
//    std::cout<<"stop_line_num="<<stop_line_num<<std::endl;



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
bool parking_space::save_ps_to_txt_file(int frame_num, now_ps_type & ps_info, std::string file_name)
{
    FILE *out_file_detect;
    if(out_file_detect = fopen(file_name.c_str(), "a+"))
    {
        puts("file open successed!");
        int n = ps_info.ps.size();
        double x1,y1,x2,y2,alpha_left,alpha_right;
        if (n>0)
        {
            for (int i = 0; i < n; ++i)
            {
                x1 = ps_info.ps[i].left_point.point_L.x;
                y1 = ps_info.ps[i].left_point.point_L.y;
                x2 = ps_info.ps[i].right_point.point_L.x;
                y2 = ps_info.ps[i].right_point.point_L.y;
                alpha_left = ps_info.ps[i].left_point.alpha;
                alpha_right = ps_info.ps[i].right_point.alpha;
                fprintf(out_file_detect,"%d,%f,%f,%f,%f,%f,%f\n",frame_num,x1,y1,x2,y2,alpha_left,alpha_right);
            }
        }
        fclose(out_file_detect);
        return 1;
    }
    else
    {
        puts("file open failed!");
        return 0;
    }
}



















