//
// Created by lx on 19-8-11.
//
#include "tools.h"


double calu_dis_2point(cv::Point &p1, cv::Point &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB)
{
    //求直线方程
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;
    //代入点到直线距离公式
    float distance = 0;
    distance = ((float)abs(A*pointP.x + B*pointP.y + C)) / (sqrtf(A*A + B*B));
    return distance;
}

double calu_contours_dis(std::vector<cv::Point> &src1, std::vector<cv::Point> &src2)
{
    double now_dis;
    double min_dis = 1e9;
    for(int i=0;i<src1.size();i=i+5)
    {
        for(int j=0;j<src2.size();j=j+5)
        {
            now_dis = calu_dis_2point(src1[i],src2[j]);
            if(now_dis < min_dis)
            {
                min_dis = now_dis;
            }
        }
    }
    return min_dis;
}
bool is_in_vector(int a,std::vector<int> v)
{
    for(int i=0;i<v.size();i++)
    {
        if(a == v[i])
        {
            return 1;
        }
    }
    return 0;
}
//轮廓距离：最近点间的欧式距离
//输入 图片中含有的总轮廓个数
//输出类别-每个类别包含的轮廓索引。
int contours_cluster(std::vector<std::vector<cv::Point>> &src,
                     std::vector<std::vector<int>> & dst)
{
    const int num = src.size();
    double score[num][num] = {0,}; //二部匹配矩阵
    for(int h=0;h<num;h++)
    {
        for(int w=h+1;w<num;w++)
        {
            score[h][w] = calu_contours_dis(src[h],src[w]);
        }
    }

    //二部计算结果
    std::vector<std::vector<int>> match_s1; //stage one match result
    std::vector<bool> isCluster(src.size(),0); //h方向是否已经分配

    for(int h=0; h<num; h++) //第i行
    {
        std::vector<int> ct;
        ct.push_back(h);
        for(int w=h+1; w<num; w++)
        {
            if(score[h][w] < 20 )
            {
                ct.push_back(w);
            }
        }
        match_s1.push_back(ct); //一类
    }

//    for(int i=0; i<match_s1.size();i++)
//    {
//        //std::cout<<"match_s1[i]="<<std::endl;
//        for(int j=0;j<match_s1[i].size();j++)
//        {
//            std::cout<<match_s1[i][j]<<" ";
//        }
//        std::cout<<endl;
//    }

    std::vector<std::vector<int>> cluser_result; //最终聚类结果
    bool now_c = 1;
    for(int i=0; i<match_s1.size();i++)
    {
        if (i == 0)
        {
            cluser_result.push_back(match_s1[i]);
        }
        else
        {
            for(int j=0; j<cluser_result.size(); j++)
            {
                if(is_in_vector(match_s1[i][0],cluser_result[j]))
                {
                    if(match_s1[i].size() > 1)
                    {
                        for(int k=1; k<match_s1[i].size(); k++)
                        {
                            cluser_result[j].push_back(match_s1[i][k]);
                        }
                    }
                    now_c = 0;
                }
            }
            if (now_c)
            {
                cluser_result.push_back(match_s1[i]);
            }
            else
            {
                now_c = 1;
            }
        }
        std::cout<<std::endl;
    }

//    for(int i=0; i<cluser_result.size();i++)
//    {
//        //std::cout<<"cluser_result[i]="<<std::endl;
//        for(int j=0;j<cluser_result[i].size();j++)
//        {
//            std::cout<<cluser_result[i][j]<<" ";
//        }
//        std::cout<<endl;
//    }
    dst = cluser_result;
    return 1;
}

//输入，输出分别为输入的轮廓(一个轮廓)，输出为凸轮廓点集(对应的一个凸轮廓点集)
int extrct_convex_points(std::vector<cv::Point> &src,std::vector<cv::Point> &dst, int min_t) //在轮廓点中，根据凸包分析，将凸部分点集提取
{
    //求凸包
    std::vector<int> hull; //保存凸包点的索引(针对轮廓点集)
    double start = static_cast<double>(cvGetTickCount());
    convexHull(src, hull, true);
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "计算凸包:" << time/1000<<"ms"<< std::endl;

    //判断是否有凸缺陷
    // 如果有凸缺陷就把它画出来
    if( isContourConvex(src) ) //判断轮廓points_01是否为凸
    {
        std::cout<<"src的轮廓是凸包"<< std::endl;
        dst = src;
        return 0;
    }
    else
    {
        std::cout<<"src的轮廓含有凹缺陷"<< std::endl;
        std::vector< cv::Vec4i> defects;
        //求凹缺陷
        start = static_cast<double>(cvGetTickCount());
        convexityDefects(src, cv::Mat(hull), defects);
        time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
        std::cout << "计算凸缺陷:" << time/1000<<"ms"<<std::endl;

        std::vector<int> points_dev_index; //凹点索引
        bool has_inv_convex = 0;// 标志 是否有满足条件的凹缺陷
        for(int i=0;i < defects.size();++i) //遍历所有的凹缺陷
        {
            //std::cout<<"凹陷的距离="<<defects[i][3]/256 << std::endl;
            if(defects[i][3]/256 > min_t)
            {
                if(defects[i][0] < defects[i][1])
                {
                    has_inv_convex = 1;
                    for(int k=defects[i][0]; k<defects[i][1];k++)
                    {
                        points_dev_index.push_back(k);
                    }
                }
            }
        }

        if (has_inv_convex)
        {
            //src轮廓索引 - 凹点集索引 = 凸点集索引
            std::vector<cv::Point> points_convex; //凸点集合
            int start_dex = 0;
            std::cout<<"contours[0].size()"<<src.size()<<std::endl;
            for(int m=0;m<src.size();m++)
            {
                if(m == points_dev_index[start_dex])
                {
                    start_dex++;
                }
                else
                {
                    points_convex.push_back(src[m]);
                }
            }
            dst = points_convex;
            return 1;  //含有满足条件的凹缺陷
        }
        else
        {
            dst = src;
            return 0;
        }
    }
}

int  ipm_points(std::vector<cv::Point> &src, std::vector<cv::Point> &dst)
{
    cv::Mat perspectiveMat(cv::Size(3, 3), CV_64FC1);
    cv::Mat shifPerspectiveMat(cv::Size(3, 3), CV_64FC1);
    const std::string perspectiveFileName = "/home/lx/data/suround_view_src_data/calibration/weishi/fish4/ipm_m.txt";
    readPerspectiveParams(perspectiveFileName, perspectiveMat, shifPerspectiveMat);
    std::vector<cv::Point2f> src_tmp,dst_tmp;
    for(int i=0; i<src.size(); i++)
    {
        src_tmp.push_back(cv::Point2f(src[i].x,src[i].y));
    }
    cv::perspectiveTransform(src_tmp, dst_tmp, perspectiveMat);
    for(int i=0; i<dst_tmp.size(); i++)
    {
        dst.push_back(cv::Point(dst_tmp[i].x,dst_tmp[i].y));
    }
    return 1;
}

int solve_mid_line(aps::LineModel &l1, aps::LineModel &l2, aps::LineModel &l_mid)
{
    double sub_k  = fabs(l1.mSlope - l2.mSlope);
    if ( sub_k <  0.0001 )
    {
        l_mid.mSlope = (l1.mSlope + l2.mSlope) / 2.0;
        l_mid.mIntercept = (l1.mIntercept + l2.mIntercept) / 2.0;
    }
    else
    {
        // 1 求交点
        double x0 = (l2.mIntercept - l1.mIntercept)/(l1.mSlope - l2.mSlope);
        double y0 = x0 * l1.mSlope + l1.mIntercept;
        // 2.求斜率
        double alpha1, alpha2, alpha3;
        alpha1 = atan(l1.mSlope);
        alpha2 = atan(l2.mSlope);
        alpha3 = (alpha1 + alpha2)/2.0;
        //kx+b方程
        l_mid.mSlope = tan(alpha3);
        l_mid.mIntercept = y0 - l_mid.mSlope * x0;
    }
    return 1;
}

































