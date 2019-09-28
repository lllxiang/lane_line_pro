//
// Created by lx on 19-8-11.
//

#include "detect_parking_space.h"

void parking_space::show()
{
    cv::Mat result = img_ps_bgr.clone();
    cv::Mat src = img_ps_bgr.clone();

    if(now_info.is_has_ps)
    {
        //滤波后的点
        for(int i=0; i< contours_filtered.size();i++)
        {
            for(int j=0;j<contours_filtered[i].size();j++)
            {
                //cv::circle(img_ps_bgr,contours_filtered[i][j],1,cv::Scalar(0,0,255),1,8,0);
            }
        }
        //显示聚类后的结果
        for(int i=0;i<con_clusters.size();i++) //每一类
        {
            cv::Scalar color(rand()&255, rand()&255,rand()&255);
            for(int j=0; j< con_clusters[i].size(); j++)
            {
                std::vector<cv::Point> tc = contours_filtered_con[con_clusters[i][j]];
                for(int m=0; m<tc.size(); m++)
                {
                    circle(img_ps_bgr, tc[m], 1, color, CV_FILLED, CV_AA);
                }
            }
        }

        //显示拟合的直线
        for(int i=0;i<rects.size();i++)
        {
            //在透视图中显示拟合的直线
            cv::Point p1(0,0*rects[i].fit_line1.mSlope + rects[i].fit_line1.mIntercept);
            cv::Point p2(640,640*rects[i].fit_line1.mSlope + rects[i].fit_line1.mIntercept);
            cv::line(img_ps_bgr, p1, p2, cv::Scalar(255, 0, 0), 1, 8);  //绘制最小外接矩形每条边
            cv::circle(img_ps_bgr, rects[i].fit_line1.p_toushi[0],3,cv::Scalar(255,0,0),1,8,0);
            cv::circle(img_ps_bgr, rects[i].fit_line1.p_toushi[1],3,cv::Scalar(255,0,0),1,8,0);

            cv::Point p3(0,0*rects[i].fit_line2.mSlope + rects[i].fit_line2.mIntercept);
            cv::Point p4(640,640*rects[i].fit_line2.mSlope + rects[i].fit_line2.mIntercept);
            cv::line(img_ps_bgr, p3, p4, cv::Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
            cv::circle(img_ps_bgr, rects[i].fit_line2.p_toushi[0],3,cv::Scalar(0,0,255),1,8,0);
            cv::circle(img_ps_bgr, rects[i].fit_line2.p_toushi[1],3,cv::Scalar(0,0,255),1,8,0);

            //mid line
            cv::Point p5(0,0*rects[i].fit_line_mid.mSlope + rects[i].fit_line_mid.mIntercept);
            cv::Point p6(640,640*rects[i].fit_line_mid.mSlope + rects[i].fit_line_mid.mIntercept);
            cv::line(img_ps_bgr, p5, p6, cv::Scalar(0, 255, 0), 1, 8);  //绘制最小外接矩形每条边
            cv::circle(img_ps_bgr, rects[i].fit_line_mid.p_toushi[0],3,cv::Scalar(0,0,255),1,8,0);
            cv::circle(img_ps_bgr, rects[i].fit_line_mid.p_toushi[1],3,cv::Scalar(255,0,0),1,8,0);

            //mid line 俯视图中画点
            cv::Point p_mid_ipm1, p_mid_ipm2;
            p_mid_ipm1 = rects[i].fit_line_mid.p_ipm[0]; p_mid_ipm2 = rects[i].fit_line_mid.p_ipm[1];

            cv::circle(img_ps_bgr_ipm, p_mid_ipm1,3,cv::Scalar(0,0,255),1,8,0);
            cv::circle(img_ps_bgr_ipm, p_mid_ipm2,3,cv::Scalar(255,0,0),1,8,0);
            cv::line(img_ps_bgr_ipm, p_mid_ipm1, p_mid_ipm2, cv::Scalar(0, 255, 0), 3  , 8);  //绘制最小外接矩形每条边



            //在俯视图中显示拟合的直线
            cv::circle(img_ps_bgr_ipm, rects[i].fit_line1.p_ipm[0],3,cv::Scalar(255,0,0),1,8,0);
            cv::circle(img_ps_bgr_ipm, rects[i].fit_line1.p_ipm[1],3,cv::Scalar(255,0,0  ),1,8,0);

            cv::circle(img_ps_bgr_ipm, rects[i].fit_line2.p_ipm[0],3,cv::Scalar(0,0,255),1,8,0);
            cv::circle(img_ps_bgr_ipm, rects[i].fit_line2.p_ipm[1],3,cv::Scalar(0,0,255),1,8,0);

            //在透视图中画出最终的车位线

            for (int i=0; i<now_info.ps.size(); i++)
            {
                cv::line(result,now_info.ps[i].sr1.fit_line_mid.p_toushi[0], now_info.ps[i].sr1.fit_line_mid.p_toushi[1],
                        cv::Scalar(255,0,0), 2, 8, 0);
                cv::line(result,now_info.ps[i].sr2.fit_line_mid.p_toushi[0], now_info.ps[i].sr2.fit_line_mid.p_toushi[1],
                         cv::Scalar(255,0,0), 2, 8, 0);
                cv::circle(result, now_info.ps[i].sr1.fit_line_mid.p_toushi[0],7,cv::Scalar(0,255,0 ),-1,8,0);
                cv::circle(result, now_info.ps[i].sr2.fit_line_mid.p_toushi[0],7,cv::Scalar(0,255,0),-1,8,0);
            }
        }

        outputVideo << result;
        cv::imshow("src", src);
        cv::imshow("result", result);
        cv::imshow("img_ps_mask", img_ps_mask);
        cv::imshow("img_ps_mask_ipm", img_ps_mask_ipm);
        cv::imshow("img_ps_bgr", img_ps_bgr);
        cv::imshow("img_ps_bgr_ipm", img_ps_bgr_ipm);
        cv::waitKey(20);

    }
    else
    {
        outputVideo << result;
        cv::imshow("src", src);
        cv::imshow("result", result);
        cv::imshow("img_ps_mask", img_ps_mask);
        cv::imshow("img_ps_mask_ipm", img_ps_mask_ipm);
        cv::imshow("img_ps_bgr", img_ps_bgr);
        cv::imshow("img_ps_bgr_ipm", img_ps_bgr_ipm);
        cv::waitKey(20);

    }

}

int parking_space::contours_filter()
{
    std::vector<cv::Vec4i> hierarchy;
    double start = static_cast<double>(cvGetTickCount());
    cv::findContours(img_ps_mask, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE );
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "1->findContours及滤波耗时:" << time/1000<<"ms"<<std::endl;

    printf("滤波前所有的轮廓 n=%d\n",(int)contours.size());
    for(int i=0;i<contours.size();i++) //对所有的轮廓
    {
        int s = cv::contourArea(contours[i]);
        int len = cv::arcLength(contours[i],1);
        if( s > 400)
        {
            contours_filtered.push_back(contours[i]);
            printf("s=%d\n",s);
            printf("len=%d\n",len);
        }
    }
    return contours_filtered.size();
}

int parking_space::detect()
{
    //形态学-膨胀
    //获取 kernel的形状
    cv::Mat ds = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9),cv::Point(-1,-1));

    double start = static_cast<double>(cvGetTickCount());
    cv::dilate(img_ps_mask, img_ps_mask, ds);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "2->膨胀耗时:" << time/1000<<"ms"<<std::endl;

    //滤波
    if (!contours_filter())
    {
        std::cout<<"无车位"<<std::endl;
        now_info.is_has_ps = 0;
        return -1;
    }

    start = static_cast<double>(cvGetTickCount());
    contours_cluster(contours_filtered, con_clusters);
    time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "3->轮廓聚类耗时:" << time/1000<<"ms"<<std::endl;

    //凸轮廓查找
    std::vector<cv::Point> dst;
    for(int i=0; i<contours_filtered.size(); i++)
    {
        start = static_cast<double>(cvGetTickCount());
        extrct_convex_points(contours_filtered[i], dst, 30);
        time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
        std::cout << "4->凹性分析耗时:" << time/1000<<"ms"<<std::endl;
        contours_filtered_con.push_back(dst);
        dst.clear();
    }

    //RANSAC拟合//ransac 拟合直线
    start = static_cast<double>(cvGetTickCount());
    for(int i=0;i<con_clusters.size();i++) //每一类
    {
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
        ransac_lines1.setRequiredInliers(int(pts.size()/5));
        ransac_lines1.setIterations(20);
        ransac_lines1.setTreshold(5);
        double start = static_cast<double>(cvGetTickCount());
        isfound_line1 = ransac_lines1.computeModel();
        double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
        std::cout << "ransac_lines1所用时间为:" << time/1000<<"ms"<<std::endl;
        //aps::LineModel fit_line;
        super_rect rc;
        ransac_lines1.getBestModel(rc.fit_line1);
        rc.fit_line1.p_toushi.push_back(ransac_lines1.p_min);
        rc.fit_line1.p_toushi.push_back(ransac_lines1.p_max);
        std::cout<<"ransac_lines1.notConsensusSet"<<ransac_lines1.m_notConsensusSet.size()<<std::endl;
        if (ransac_lines1.m_notConsensusSet.size() > 6 )
        {
            aps::RansacLine2D ransac_lines2;
            ransac_lines2.setObservationSet(ransac_lines1.m_notConsensusSet);
            ransac_lines2.setRequiredInliers(int(pts.size()/5));
            ransac_lines2.setIterations(20);
            ransac_lines2.setTreshold(5);
            start = static_cast<double>(cvGetTickCount());
            isfound_line2 = ransac_lines2.computeModel();
            time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
            std::cout << "ransac_lines2所用时间为:" << time/1000<<"ms"<<std::endl;
            ransac_lines2.getBestModel(rc.fit_line2);
            rc.fit_line2.p_toushi.push_back(ransac_lines2.p_min);
            rc.fit_line2.p_toushi.push_back(ransac_lines2.p_max);
        }

        //只有当两条线都找到，且  两条线的斜率夹角满足一定条件  ，才转换至俯视图中。
        //rc 中两条线段的端点 转换至 ipm坐标系下
        // 对车位引导线进行较强的约束
        if (isfound_line1 && isfound_line2)
        {
            //rc中 根据检测出的两条线求 角平分线。。只有在同时检测出两条线的时候调用
            solve_mid_line(rc.fit_line1, rc.fit_line2, rc.fit_line_mid);

            //求 中间线段的端点
            bool is_start =  0;
            for (int y0=480; y0>0; y0=y0-1  )
            {
                double x0 = (y0 - rc.fit_line_mid.mIntercept) /rc.fit_line_mid.mSlope;
                if (x0<0 || x0>640)
                { continue;}

                if ((img_ps_mask.at<uchar>(y0, x0) == 255) && !is_start)
                {
                    //保存此时的 x0,y0
                    rc.fit_line_mid.p_toushi.push_back(cv::Point(int(x0),y0));
                    is_start = 1;
                }
                if ((img_ps_mask.at<uchar>(y0, x0) == 0) && is_start )
                {
                    //保存此时的 x0,y0
                    rc.fit_line_mid.p_toushi.push_back(cv::Point(int(x0),y0));
                    break;
                }
            }
            if (rc.fit_line_mid.p_toushi.size() == 0)
            {
                rc.fit_line_mid.p_toushi.push_back(cv::Point(int(0),0));
                rc.fit_line_mid.p_toushi.push_back(cv::Point(int(0),0));
            }
            else if (rc.fit_line_mid.p_toushi.size() == 1)
            {
                rc.fit_line_mid.p_toushi.push_back( rc.fit_line_mid.p_toushi[0]);
            }
            //将中间线段、两侧线段的端点坐标转移至 俯视图坐标系下
            ipm_points(rc.fit_line1.p_toushi, rc.fit_line1.p_ipm);
            ipm_points(rc.fit_line2.p_toushi, rc.fit_line2.p_ipm);
            ipm_points(rc.fit_line_mid.p_toushi, rc.fit_line_mid.p_ipm);
            //求俯视图下 线段的斜率，与x轴正方向的夹角
            rc.fit_line_mid.mSlope_ipm = (rc.fit_line_mid.p_ipm[1].y - rc.fit_line_mid.p_ipm[0].y) / (rc.fit_line_mid.p_ipm[1].x - rc.fit_line_mid.p_ipm[0].x + 1e-10);
            rc.fit_line_mid.mIntercept_ipm = rc.fit_line_mid.p_ipm[0].y - rc.fit_line_mid.mSlope_ipm * rc.fit_line_mid.p_ipm[0].x;

            rc.fit_line_mid.alpha_ipm = atan(rc.fit_line_mid.mSlope_ipm);
            if (rc.fit_line_mid.alpha_ipm < 0)
            {
                rc.fit_line_mid.alpha_ipm = pi + rc.fit_line_mid.alpha_ipm;
            }
            rects.push_back(rc); //rects 表示一帧图像中所有的超矩形。每个超举行由两条线段，中间线段原始组成
            //找完所有的超矩形，在俯视图中 对中间线段进行聚类。
            //输入为   std::vector<super_rect> rects; ,输出为 ps_one_frame now_info;
            find_parking_space();
        }
    }
    time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "5->RANSAC直线拟合及车位查找耗时:" << time/1000<<"ms"<<std::endl;
    if (now_info.ps.size() == 0)
    {
        now_info.is_has_ps = 1;
    }
    else
    {
        now_info.is_has_ps = now_info.ps.size();
        //after find parking space, find p_t length.
        for(int i=0; i< now_info.ps.size();i++)
        {
            cv::Point pt_s, pt_e;
            pt_s = now_info.ps[i].sr1.fit_line_mid.p_ipm[0];
            for(int y = now_info.ps[i].sr1.fit_line_mid.p_ipm[0].y; y>0; y--)
            {
                double x = (y - now_info.ps[i].sr1.fit_line_mid.mIntercept_ipm) / (now_info.ps[i].sr1.fit_line_mid.mSlope_ipm + 1e-10);
                pt_e = cv::Point(int(x),y);
                double dis  = calu_dis_2point(pt_s, pt_e);
                if (dis > 100)
                {
                    now_info.ps[i].sr1.fit_line_mid.p_ipm[0] = pt_s;
                    now_info.ps[i].sr1.fit_line_mid.p_ipm[1] = pt_e;
                    break;
                }
            }
        }
    }

    return 1;
}


int parking_space::detect_test() {
    std::vector<cv::Scalar> cmap;
    for(int i=0;i<50;i++)
    {
        cmap.push_back(cv::Scalar(rand()%255, rand()%255, rand()%255));
    }
    cv::Mat img_t = cv::Mat::zeros(img_ps_mask_ipm.size(), CV_8UC1);
    std::vector<cv::Vec4i> hierarchy;
    double start = static_cast<double>(cvGetTickCount());
    cv::findContours(img_ps_mask_ipm, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE );
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "1->findContours及滤波耗时:" << time/1000<<"ms"<<std::endl;

    for (int i=0;i<contours.size();i++)
    {
        for(int j=0; j<contours[i].size();j++)
        {
            img_t.at<uchar>(contours[i][j].y, contours[i][j].x) = 255;
        }
    }

    printf("滤波前所有的轮廓 n=%d\n",(int)contours.size());



    cv::Canny(img_ps_mask_ipm, img_mask_canndy, 0.1,0.5); //0.8-1.1ms

    std::vector<cv::Vec4f> plines;
    //Hough直线检测API
     start = static_cast<double>(cvGetTickCount());
     cv::HoughLinesP(img_t, plines, 1, CV_PI / 180, 30, 40, 10);
     time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "1->hough边缘检测耗时:" << time/1000<<"ms"<<std::endl;


    //标记出直线
    for (size_t i = 0; i < plines.size(); i++)
    {
        cv::Vec4f point1 = plines[i];
        if (point1[0] == point1[2])
        {
            cv::circle(img_ps_bgr_ipm,cv::Point(point1[0], point1[1]),2,cv::Scalar(0,0,255),1,8,0 );
            cv::circle(img_ps_bgr_ipm,cv::Point(point1[2], point1[3]),2,cv::Scalar(0,255,0),1,8,0 );
        }

        line(img_ps_bgr_ipm, cv::Point(point1[0], point1[1]), cv::Point(point1[2], point1[3]), cv::Scalar(rand()%255, rand()%255, rand()%255), 1, cv::LINE_AA);
    }

    //输入plines, 输出plines_combined
    std::vector<cv::Vec4f> plines_combined;
    lines_combined(plines,plines_combined);

    for(int i=0; i<plines_combined.size();i++)
    {
        cv::Vec4f point1 = plines_combined[i];
        line(img_ps_bgr_ipm, cv::Point(point1[0], point1[1]), cv::Point(point1[2], point1[3]), cmap[i], 2, cv::LINE_AA);
    }


    cv::imshow("img_t", img_t);
    cv::imshow("img_ps_mask", img_ps_mask);
    cv::imshow("img_ps_bgr", img_ps_bgr);
    cv::imshow("img_ps_bgr_ipm", img_ps_bgr_ipm);
    cv::imshow("img_mask_canndy", img_mask_canndy);
    cv::waitKey(0);
}



int parking_space::find_parking_space() {
    //输入为   std::vector<super_rect> rects; ,输出为 ps_one_frame now_info;
    //find_parking_space();
    //根据距离约束、平行度约束、起点偏移三个特征进行聚类。 每一个车位信息结构体
    // 包含两条线，代表一个车位存在的位置，
    // 孤立的引导线被删除
    int i=0; //考察第0个引导线
    double disptol=0, para=0;
    double start_dis = 0;
    printf("rects.size()=%d\n",(int)rects.size());
    if (rects.size() > 1)
    {
        for(int j=0; j<rects.size();j++)
        {
            printf("j: rects[j].fit_line_mid.p_ipm[0]: %d ,x= %d, y= %d \n", j, rects[j].fit_line_mid.p_ipm[0].x, rects[j].fit_line_mid.p_ipm[0].y);
            for (int i = j+1; i < rects.size(); i++)
            {
                //printf("i: l_edge_point: %d ,x= %d, y= %d \n", i, box[i].l_edge_point[0].x, box[i].l_edge_point[0].y);
                //printf("index=%d, x=%d \n", i, box[i].l_edge_point[0].x);
                disptol = getDist_P2L(rects[j].fit_line_mid.p_ipm[0], rects[i].fit_line_mid.p_ipm[0],
                                      rects[i].fit_line_mid.p_ipm[1]);
                double alpha1 = rects[j].fit_line_mid.alpha_ipm;
                double alpha2 = rects[i].fit_line_mid.alpha_ipm;
                para = abs(abs(alpha1) - abs(alpha2));

                start_dis = (rects[j].fit_line_mid.p_ipm[0].y -  rects[i].fit_line_mid.p_ipm[0].y);

                printf("i = %d disptol=%f,para=%f, start_dis=%f \n",i,  disptol, para, start_dis);
                if ((disptol > 110 && disptol < 150) && (para < 5) && (start_dis < 50))
                {
                    parking_space_info tmp;
                    //将两条线信息->车位信息。
                    //cv::line(src_img_ipm, )
                    cv::Scalar c = cv::Scalar(rand()%255, rand()%255, rand()%255);
                    tmp.sr1 = rects[j];
                    tmp.sr2 = rects[i];
                    now_info.ps.push_back(tmp);

                    //cv::line(img_ps_bgr_ipm,tmp.sr1.fit_line_mid.p_ipm[0], tmp.sr1.fit_line_mid.p_ipm[1],c, 2, 8, 0);
                    //cv::line(img_ps_bgr_ipm,tmp.sr2.fit_line_mid.p_ipm[0], tmp.sr2.fit_line_mid.p_ipm[1],c, 2, 8, 0);
                    //cv::imshow("img_ps_bgr_ipm2", img_ps_bgr_ipm);
                    //cv::waitKey(0);
                }
            }
        }
    }
    else
    {
        return 0;
    }
}

parking_space::parking_space() {
    cv::Size sWH = cv::Size(640,480);
    std::string outputVideoPath = "../test.avi";
    outputVideo.open(outputVideoPath, CV_FOURCC('M', 'P', '4', '2'), 25.0, sWH);
}
parking_space::~parking_space() {
    outputVideo.release();
}
