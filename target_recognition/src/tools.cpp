//
// Created by lx on 19-8-11.
//
#include "tools.h"

double calu_dis_2point(cv::Point &p1, cv::Point &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
double calu_dis_2point2f(cv::Point2f &p1, cv::Point2f &p2)
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

double calu_dis_2lines(cv::Vec4f & line1, cv::Vec4f & line2)
{
    double atob = getDist_P2L(cv::Point(line1[0], line1[1]), cv::Point(line2[0], line2[1]), cv::Point(line2[2], line2[3]));
    double btoa = getDist_P2L(cv::Point(line2[0], line2[1]), cv::Point(line1[0], line1[1]), cv::Point(line1[2], line1[3]));
    return (atob+btoa)/2.0f;
}
bool polynomial_curve_fit(std::vector<cv::Point>& points, int rank, cv::Mat& coef)
{
    //Number of points
    int num = points.size();
    //X matrix
    cv::Mat X = cv::Mat::zeros(rank + 1, rank + 1, CV_64FC1);
    for (int i = 0; i < rank + 1; i++)
    {
        for (int j = 0; j < rank + 1; j++)
        {
            for (int k = 0; k < num; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                                     std::pow(points[k].x, i + j);
            }
        }
    }
    //Y matrix
    cv::Mat Y = cv::Mat::zeros(rank + 1, 1, CV_64FC1);
    for (int i = 0; i < rank + 1; i++)
    {
        for (int k = 0; k < num; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                                 std::pow(points[k].x, i) * points[k].y;
        }
    }
    coef = cv::Mat::zeros(rank + 1, 1, CV_64FC1);
    //solve A
    bool is_work = cv::solve(X, Y, coef, CV_LU);
    //std::cout << "is_work = " << is_work << std::endl;
    //std::cout << "coef.at<double>(0,0) = " << coef.at<double>(0,0) << std::endl;
    //std::cout << "coef.at<double>(1,0) = " << coef.at<double>(1,0) << std::endl;
    return true;
}

double calu_alpha_line(cv::Vec4f & line, bool is_)
{
    //check over
    if (line[1] == line[3])
    {
        return 0; //水平
    }
    else if (line[0] == line[2]) //垂直
    {
        if(is_)
        {
            return CV_PI/2.0;
        }
        else
        {
            return 90;
        }
    }
    else
    {
        double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-10);
        double alpha = atan(k);
        if (alpha < 0)
        {
            alpha =  - alpha;
        }
        else
        {
            alpha = CV_PI - alpha ;
        }

        if(is_)
        {
            return alpha;
        }
        else
        {
            return alpha * 180 / CV_PI;
        }
    }
}
double calu_alpha_line(cv::Vec6f & line, bool is_)
{
    //check over
    if (line[1] == line[3])
    {
        return 0; //水平
    }
    else if (line[0] == line[2]) //垂直
    {
        if(is_)
        {
            return CV_PI/2.0;
        }
        else
        {
            return 90;
        }
    }
    else
    {
        double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-10);
        double alpha = atan(k);
        if (alpha < 0)
        {
            alpha =  - alpha;
        }
        else
        {
            alpha = CV_PI - alpha ;
        }

        if(is_)
        {
            return alpha;
        }
        else
        {
            return alpha * 180 / CV_PI;
        }
    }
}

double calu_para_2lines(cv::Vec4f & line1, cv::Vec4f & line2)
{
    cv::Vec4f line1_(line1[0], line1[1], line1[2], line1[3]);
    cv::Vec4f line2_(line2[0], line2[1], line2[2], line2[3]);
    double para = abs(calu_alpha_line(line1_, 0) - calu_alpha_line(line2_, 0));
    if (para <= 90)
    {
        return para;
    }
    else
    {
        return 180 - para; //夹角较大时，取临角
    }
}
double calu_para_2lines(cv::Vec6f & line1, cv::Vec6f & line2)
{
    cv::Vec4f line1_(line1[0], line1[1], line1[2], line1[3]);
    cv::Vec4f line2_(line2[0], line2[1], line2[2], line2[3]);
    double para = abs(calu_alpha_line(line1_, 0) - calu_alpha_line(line2_, 0));
    if (para < 90)
    {
        return para;
    }
    else
    {
        return 180 - para; //夹角较大时，取临角
    }
}
double calu_dis_2lines_m2(cv::Vec4f & line1, cv::Vec4f & line2)
{
    cv::Point2f p1_s(line1[0],line1[1]);
    cv::Point2f p1_e(line1[2],line1[3]);
    cv::Point2f p2_s(line2[0],line2[1]);
    cv::Point2f p2_e(line2[2],line2[3]);

    double dis1 = calu_dis_2point2f(p1_s, p1_e);
    double dis2 = calu_dis_2point2f(p2_s, p2_e);
    if (dis1 < dis2)
    {
        //getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
        return (getDist_P2L(cv::Point(p1_s),cv::Point(p2_s),cv::Point(p2_e)) +
        getDist_P2L(cv::Point(p1_e),cv::Point(p2_s),cv::Point(p2_e))) / 2.0f;
    }
    else
    {
        return (getDist_P2L(cv::Point(p2_s),cv::Point(p1_s),cv::Point(p1_e)) +
                getDist_P2L(cv::Point(p2_e),cv::Point(p1_s),cv::Point(p1_e))) / 2.0f;
    }
}
double calu_dis_2lines_m2(cv::Vec6f & line1, cv::Vec6f & line2)
{
    cv::Point2f p1_s(line1[0],line1[1]);
    cv::Point2f p1_e(line1[2],line1[3]);
    cv::Point2f p2_s(line2[0],line2[1]);
    cv::Point2f p2_e(line2[2],line2[3]);

    double dis1 = calu_dis_2point2f(p1_s, p1_e);
    double dis2 = calu_dis_2point2f(p2_s, p2_e);
    if (dis1 < dis2)
    {
        //getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
        return (getDist_P2L(cv::Point(p1_s),cv::Point(p2_s),cv::Point(p2_e)) +
                getDist_P2L(cv::Point(p1_e),cv::Point(p2_s),cv::Point(p2_e))) / 2.0f;
    }
    else
    {
        return (getDist_P2L(cv::Point(p2_s),cv::Point(p1_s),cv::Point(p1_e)) +
                getDist_P2L(cv::Point(p2_e),cv::Point(p1_s),cv::Point(p1_e))) / 2.0f;
    }
}



double solve_theta_d(double theta_d,cv::Mat & D)
{
    double D1 = D.at<double>(0,0);
    double D2 = D.at<double>(1,0);
    double D3 = D.at<double>(2,0);
    double D4 = D.at<double>(3,0);

    //考虑第一阶误差，以theta_d为初始值，迭代求解方程 theta_d = theta ( 1 + D1*theta^2 )
    double initial_theta = theta_d;
    double deta_theta=1;
    double error = -theta_d + initial_theta*(1+D.at<double>(0,0)*pow(initial_theta,2));
    while(fabs(error)>1e-8 && fabs(deta_theta)>1e-20)//设定残差阈值1e-8，步长阈值1e-20,缩小此参数可进一步提高精度
    {
        double theta_old = initial_theta;
        double error_d = initial_theta * ( 2*D1*initial_theta ) + ( 1 + D1*pow(initial_theta,2) );
        if (error_d < 1e-10)
        {
            std::cerr<<"wrong error_d"<<std::endl;
            break;
        }
        initial_theta = initial_theta - error/error_d;
        error = -theta_d + initial_theta*(1+D.at<double>(0,0)*pow(initial_theta,2) );
        deta_theta = fabs(theta_old - initial_theta);
    }

    //以之前求出的theta作为初始值，迭代求解方程 theta_d = theta ( 1 + D1*theta^2 + D2*theta^4 + D3*theta^6 + D4*theta^8 )
    double theta = initial_theta;
    error = -theta_d + theta*(1+D.at<double>(0,0)*pow(theta,2) + D.at<double>(1,0)*pow(theta,4) + D.at<double>(2,0)*pow(theta,6) + D.at<double>(3,0)*pow(theta,8) );
    deta_theta = 1;
    while(fabs(error)>1e-8 && deta_theta>1e-20)//设定残差阈值1e-8，步长阈值1e-20,缩小此参数可进一步提高精度
    {
        double theta_old = theta;
        double error_d = theta * ( 2*D1*theta + 4*D2*pow(theta,3) + 6*D3*pow(theta,5) + 8*D4*pow(theta,7) ) + ( 1 + D1*pow(theta,2) + D2*pow(theta,4) + D3*pow(theta,6) + D4*pow(theta,8) );
        if (error_d < 1e-10)
        {
            std::cerr<<"wrong error_d"<<std::endl;
            break;
        }
        theta = theta - error/error_d;
        error = -theta_d + theta*(1+D.at<double>(0,0)*pow(theta,2) + D.at<double>(1,0)*pow(theta,4) + D.at<double>(2,0)*pow(theta,6) + D.at<double>(3,0)*pow(theta,8) );
        deta_theta = fabs(theta_old - theta);
    }

    return theta;
}

void computeWorldcoordWithZ (std::vector<cv::Point3f>& objectPoints, std::vector<cv::Point2f>& imagePoints, std::vector<double>& Z,cv::Mat & rvec,
                             cv::Mat & tvec, cv::Mat & K, cv::Mat & D)
{

    cv::Mat Ma = (cv::Mat_<double>(2,2));
    cv::Mat Mb = (cv::Mat_<double>(2,1));
    cv::Mat  c = (cv::Mat_<double>(2,1));

    for (int i = 0; i < imagePoints.size(); i++)
    {
        //std::cout<<"imageposition "<<imagePoints[i].x<<" "<<imagePoints[i].y<<std::endl;

        double u = imagePoints[i].x;
        double v = imagePoints[i].y;
        double fx = K.at<double>(0,0);
        double cx = K.at<double>(0,2);
        double fy = K.at<double>(1,1);
        double cy = K.at<double>(1,2);
        double theta_d = sqrt( (u-cx)/fx * (u-cx)/fx  + (v-cy)/fy * (v-cy)/fy );

        double theta;
        theta = solve_theta_d(theta_d,D);

        double r = tan(theta);
        imagePoints[i].x = (u-cx)*r/theta_d + cx;
        imagePoints[i].y = (v-cy)*r/theta_d + cy;

        Ma.at<double>(0, 0) = K.at<double>(0, 0) * rvec.at<double>(0, 0) + K.at<double>(0, 2) * rvec.at<double>(2, 0) - rvec.at<double>(2,0) * imagePoints[i].x ;
        Ma.at<double>(0, 1) = K.at<double>(0, 0) * rvec.at<double>(0, 1) + K.at<double>(0, 2) * rvec.at<double>(2, 1) - rvec.at<double>(2,1) * imagePoints[i].x ;

        Ma.at<double>(1, 0) = K.at<double>(1, 1) * rvec.at<double>(1, 0) + K.at<double>(1, 2) * rvec.at<double>(2, 0) - rvec.at<double>(2,0) * imagePoints[i].y ;
        Ma.at<double>(1, 1) = K.at<double>(1, 1) * rvec.at<double>(1, 1) + K.at<double>(1, 2) * rvec.at<double>(2, 1) - rvec.at<double>(2,1) * imagePoints[i].y ;

        Mb.at<double>(0, 0) = tvec.at<double>(2,0) * imagePoints[i].x - K.at<double>(0, 0) * tvec.at<double>(0, 0) - K.at<double>(0, 2) * tvec.at<double>(2, 0) - ( K.at<double>(0, 0) * rvec.at<double>(0, 2) + K.at<double>(0, 2) * rvec.at<double>(2, 2) - rvec.at<double>(2,2) * imagePoints[i].x) * Z[i];//objectPoints[i].z ;
        Mb.at<double>(1, 0) = tvec.at<double>(2,0) * imagePoints[i].y - K.at<double>(1, 1) * tvec.at<double>(1, 0) - K.at<double>(1, 2) * tvec.at<double>(2, 0) - ( K.at<double>(1, 1) * rvec.at<double>(1, 2) + K.at<double>(1, 2) * rvec.at<double>(2, 2) - rvec.at<double>(2,2) * imagePoints[i].y) * Z[i];//objectPoints[i].z ;

        cv::solve(Ma, Mb, c, CV_SVD);

        objectPoints.push_back( cv::Point3f( c.at<double>(0, 0), c.at<double>(1, 0), Z[i] ));
        //std::cout<<"objectPoints "<<c.at<double>(0, 0)<<" "<<c.at<double>(1, 0)<<std::endl;
    }
}

//根据相机参数计算IPM H 及 H_inv
bool img_undistort2topview(cv::Mat & inter_params,
                           std::vector<double> distortion,
                           cv::Mat & rvecs,
                           cv::Mat & tvecs,
                           std::string direction,
                           cv::Mat & img_undistorted,
                           cv::Mat & img_ipm)
{
    cv::Mat img_distorted = cv::Mat::zeros(cv::Size(640,480), CV_8UC1);
    const double car_len = 500.5;    // Vehicle length
    const double car_weight = 193.2; //Vehicle width
    const double car_zhouju = 187.5; // Vehicle wheelbase
    const double view_dis = 500.0; //view distance 800cm

    //定义点
    std::vector<cv::Point2f> points_distort;    //points in img (distorted) 有畸变
    std::vector<cv::Point2f> points_undistort;  //points in img (undistorted) 无畸变
    std::vector<cv::Point3f> wordPoints3D;  //俯视图 3D temp
    std::vector<cv::Point2f> wordPoints2D; //俯视图 2D
    cv::Size ipm_size;
    cv::Mat H, H_inv; //IPM Mat
    points_distort.push_back(cv::Point2f(140.0,150.0));
    points_distort.push_back(cv::Point2f(465.0,157.0));
    points_distort.push_back(cv::Point2f(533.0,351.0));
    points_distort.push_back(cv::Point2f(110.0,322.0));
    cv::fisheye::undistortPoints(points_distort,points_undistort,inter_params,distortion,inter_params);

    for(int i=0; i<4;i++)
    {
        cv::circle(img_distorted,points_distort[i],2,cv::Scalar(0,0,255),1,8,0 );
        //cv::circle(img_undistorted,points_undistort[i],2,cv::Scalar(255,0,255),1,8,0 );
    }

    cv::Mat R; //旋转矩阵
    cv::Rodrigues(rvecs, R);
    std::vector<double> Z; Z.push_back(0.0); Z.push_back(0.0); Z.push_back(0.0); Z.push_back(0.0);
    std::vector<cv::Point2f> points_distort_tmp = points_distort;
    cv::Mat distCoeffs(cv::Size(1,4),CV_64F);
    for(int i=0; i<4; i++)
    {
        distCoeffs.at<double>(i, 0) = distortion[i];
    }
    computeWorldcoordWithZ(wordPoints3D, points_distort_tmp, Z,R,
                           tvecs, inter_params, distCoeffs);

    //leftd
    for(int i=0; i<4; i++)
    {
        wordPoints2D.push_back(cv::Point2f(wordPoints3D[i].x, wordPoints3D[i].y));
    }
    for(int i=0; i<4; i++)
    {
        if (direction == "left")
        {
            // 106  103 由标定板至 车轴距离决定
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + (car_len-car_zhouju)/2 + 106))/2;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis - 103))/2;
            ipm_size = cv::Size((view_dis*2 + car_len)/2,  (view_dis+car_weight/2)/2);
        }
        else if (direction == "rear")
        {
            //如果是后方向的
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + car_weight/2 - 50))/2;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis + (car_len-car_zhouju)/2) - 210)/2;
            ipm_size = cv::Size((view_dis*2 + car_weight)/2,  (view_dis+car_len/2)/2);
        }
    }
    if (1)
    {
        //H = cv::getPerspectiveTransform(points_undistort, wordPoints2D);
        H = cv::findHomography( points_undistort, wordPoints2D);
        cv::Mat H_inv = cv::findHomography( wordPoints2D, points_undistort);

       // std::cout<<"H"<<H<<std::endl;
        //std::cout<<"H_inv"<<H_inv<<std::endl;

        cv::warpPerspective(img_undistorted, img_ipm, H, ipm_size);
    }
    return 1;
}


bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id)
{
    for(int h=0;h<src.rows; ++h)
    {
        for(int w=0;w<src.cols; ++w)
        {
            if (src.at<uchar>(h,w) == id || src.at<uchar>(h,w) == 7) //|| src.at<uchar>(h,w) == 7
            {
                dst.at<uchar>(h,w) = 255;
            }
            else
            {
                dst.at<uchar>(h,w) = 0;
            }
        }
    }
    return true;
}

bool combined_line_cluster(std::vector<cv::Vec4f> & plines_combined,
                                std::vector<std::vector<cv::Vec4f>> &separating_lines)
{
    int is_cluster[plines_combined.size()]; //clustered or not
    int class_nums[plines_combined.size()]; //class number for every lines

    for(int i=0; i<plines_combined.size();++i)
    {
        class_nums[i]=10000;
        is_cluster[i] = 0;
    }

    int class_now = 0;
    for(int i=0; i<plines_combined.size(); ++i)
    {
        if(is_cluster[i] == 0) //if this line is not cluster
        {
            is_cluster[i] = 1;
            class_nums[i] = class_now;
            for (int j=0; j<plines_combined.size();j++)
            {

                if (is_cluster[j] == 0) //只处理还没聚类的
                {
                    double dis = calu_dis_2lines_m2(plines_combined[i], plines_combined[j]);
                    double param = calu_para_2lines(plines_combined[i], plines_combined[j]);
                    dis = abs(dis);
                    // std::cout << "dis= " << dis << "  param= " << param << std::endl;
                    if(dis < 3 && dis >= 0  && param < 1) //< 6cm, 且 < 1度
                    {
                        is_cluster[j] = 1;
                        class_nums[j] = class_now;
                    }
                }
                else
                {
                    ;
                }
            }
            class_now++;
        }
        else
        {
            continue;
        }
    }
    for(int i=0; i<class_now; ++i) //for every class
    {
        std::vector<cv::Vec4f> now_cluster;
        for (int j=0; j<plines_combined.size(); ++j)
        {
            if (i == class_nums[j]) //找到了配对的引导线
            {
                now_cluster.push_back(plines_combined[j]);
            }
        }
        //std::cout<<"引导线条数"<<now_cluster.size()<<std::endl;
        separating_lines.push_back(now_cluster);
    }
}

//bool classify_hough_lines(std::vector<cv::Vec4f> & plines,
//                          cv::Mat & img_ps_mask_ipm,
//                          int search_len,
//                          double black_t, //0值比例阈值
//                          std::vector<cv::Vec4f> & pos_lines,
//                          std::vector<cv::Vec4f> & neg_lines)
//{
//    for(int i=0; i<plines.size(); i++)
//    {
//        double alpha = calu_alpha_line(plines[i], 0);
//        std::cout<<"alpha="<< alpha<<std::endl;
//        if ((alpha >= 0 && alpha < 45) || (alpha >= 135 && alpha < 180)) //x
//        {
//            int top_0_sum = 0;
//            int top_255_sum = 0;
//            int bottom_0_sum = 0;
//            int bottom_255_sum = 0;
//            cv::Point mid_line((plines[i][0] + plines[i][2])/2.0, (plines[i][1] + plines[i][3])/2.0);
//            for(int y=mid_line.y - search_len; y < mid_line.y; y=y+1)
//            {
//                if(img_ps_mask_ipm.at<uchar>(y, mid_line.x) == 0)
//                {
//                    top_0_sum++;
//                }
//                else
//                {
//                    top_255_sum++;
//                }
//            }
//
//            for(int y=mid_line.y; y < mid_line.y + search_len; y=y+1)
//            {
//                if(img_ps_mask_ipm.at<uchar>(y, mid_line.x) == 0)
//                {
//                    bottom_0_sum++;
//                }
//                else
//                {
//                    bottom_255_sum++;
//                }
//            }
//            double t_top = (top_0_sum*1.0 / (top_0_sum + top_255_sum ));
//            double t_bottom = (bottom_0_sum*1.0 / (bottom_0_sum + bottom_255_sum ));
//
//            if ( t_top > 0.9)
//            {
//                pos_lines.push_back(plines[i]);
//            }
//            if (t_bottom > 0.9)
//            {
//                neg_lines.push_back(plines[i]);
//            }
//
//        }
//        else if (alpha >= 45 && alpha < 135) //y
//        {
//            int left_0_sum = 0;
//            int left_255_sum = 0;
//            int right_0_sum = 0;
//            int right_255_sum = 0;
//            cv::Point mid_line((plines[i][0] + plines[i][2])/2.0, (plines[i][1] + plines[i][3])/2.0);
//            for(int x=mid_line.x - search_len; x < mid_line.x; x=x+1)
//            {
//                if(img_ps_mask_ipm.at<uchar>(mid_line.y, x) == 0)
//                {
//                    left_0_sum++;
//                }
//                else
//                {
//                    left_255_sum++;
//                }
//            }
//            for(int x=mid_line.x; x < mid_line.x + search_len; x=x+1)
//            {
//                if(img_ps_mask_ipm.at<uchar>(mid_line.y, x) == 0)
//                {
//                    right_0_sum++;
//                }
//                else
//                {
//                    right_255_sum++;
//                }
//            }
//
//            //0.7 阈值。 一定搜索距离内, 0 像素占据 所有像素的比例，大于0.7 则认为
//            double t_left = (left_0_sum*1.0 / (left_0_sum + left_255_sum ));
//            double t_right = (right_0_sum*1.0 / (right_0_sum + right_255_sum ));
//            std::cout<<"............"<<std::endl;
//            std::cout << "left_0_sum" << left_0_sum << std::endl;
//            std::cout << "left_255_sum" << left_255_sum << std::endl;
//            std::cout << "right_0_sum" << right_0_sum << std::endl;
//            std::cout << "right_255_sum" << right_255_sum << std::endl;
//
//            std::cout << "t_left" << t_left << std::endl;
//            std::cout << "t_right" << t_right << std::endl;
//            if (  t_left > 0.8 && t_right < 0.8)
//            {
//                pos_lines.push_back(plines[i]);
//            }
//            if (  t_right > 0.8 && t_left < 0.8)
//            {
//                neg_lines.push_back(plines[i]);
//            }
//        }
//    }
//    return true;
//}
/* * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 根据采样距离, 线段, 方向标志位, 求采样举行的顶点
 * input： type 1-top； 2--bottom， 3--left， 4--right
 * return：
 * * * * * * * * * * * * * * * * * * * * * * * * * * */
bool get_rect_points(cv::Vec4f & line,
                     cv::Mat & im,
                     int d,
                     int type,
                     std::vector<cv::Point> & pts)
{
    pts.clear();
    double alpha = calu_alpha_line(line, 0);
    if (type == 1 || type == 2)
    {
        if (abs(alpha - 90) < 1 )
        {
            std::cout << "get_rect_points() 参数输入错误.type == 1 || type == 2" <<std::endl;
            return -1; //求垂直直线的上下矩形， 异常退出
        }
    }
    if (type == 3 || type == 4)
    {
        if (abs(alpha - 0) < 1  || abs(alpha - 180) < 1)
        {
            std::cout << "get_rect_points() 参数输入错误.type == 3 || type == 4" <<std::endl;
            return -1; //求垂直直线的上下矩形， 异常退出
        }
    }


    if (type == 1) //上矩形采样
    {
        double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-15);
        double k_ = -1 / (k + 1e-15);
        double b_ = line[1] - k_ * line[0]; //y - kx
        for (int m = line[1] - 40; m < line[1]; m++)
        {
            int n =  (m - b_)/(k_+1e-10);    //x = (y - b)/k
            cv::Point pt1(n, m);
            cv::Point pt2(line[0], line[1]);
            if (calu_dis_2point(pt1, pt2) < d)
            {
                pts.push_back(pt2);
                pts.push_back(pt1);
                break;
            }
        }
        b_ = line[3] - k_ * line[2];
        for (int m = line[3] - 40; m < line[3]; m++) {
            int n = (m - b_)/(k_+1e-10);
            cv::Point pt1(n, m);
            cv::Point pt2(line[2], line[3]);
            if (calu_dis_2point(pt1, pt2) < d) {
                pts.push_back(pt1);
                pts.push_back(pt2);
                break;
            }
        }
    }
    else if (type == 2) //下矩形采样
    {
        double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-15);
        double k_ = -1 / (k + 1e-15);
        double b_ = line[1] - k_ * line[0]; //y - kx
        for (int m = line[1]; m < line[1] + 40; m++)
        {
            int n =  (m - b_)/(k_+1e-10);    //x = (y - b)/k
            cv::Point pt1(n, m);
            cv::Point pt2(line[0], line[1]);
            if (calu_dis_2point(pt1, pt2)  > d)
            {
                pts.push_back(pt2);
                pts.push_back(pt1);
                break;
            }
        }
        b_ = line[3] - k_ * line[2];
        for (int m = line[3]; m < line[3] + 40; m++) {
            int n = (m - b_)/(k_+1e-10);
            cv::Point pt1(n, m);
            cv::Point pt2(line[2], line[3]);
            if (calu_dis_2point(pt1, pt2) > d ) {
                pts.push_back(pt1);
                pts.push_back(pt2);
                break;
            }
        }
    }
    else if (type == 3) //左
    {
        double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-15);
        double k_ = -1 / (k + 1e-15);
        double b_ = line[1] - k_ * line[0];
        for (int m = line[0] - 40; m < line[0]; m++) {
            int n = k_ * m + b_;
            cv::Point pt1(m, n);
            cv::Point pt2(line[0], line[1]);
            if (calu_dis_2point(pt1, pt2) < d) {
                pts.push_back(pt2);
                pts.push_back(pt1);
                break;
            }
        }
        // 角点2
        b_ = line[3] - k_ * line[2];
        for (int m = line[2] - 40; m < line[2]; m++)
        {
            int n = k_ * m + b_;
            cv::Point pt1(m, n);
            cv::Point pt2(line[2], line[3]);
            if (calu_dis_2point(pt1, pt2) < d) {
                pts.push_back(pt1);
                pts.push_back(pt2);
                break;
            }
        }
    }
    else if (type ==4) //右
    {
        // 角点1
        double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-15);
        double k_ = -1 / (k + 1e-15);
        double b_ = line[1] - k_ * line[0];
        for (int m = line[0]; m < line[0] + 40; m++) {
            int n = k_ * m + b_;
            cv::Point pt1(m, n);
            cv::Point pt2(line[0], line[1]);
            if (calu_dis_2point(pt1, pt2) > d) {
                pts.push_back(pt2);
                pts.push_back(pt1);
                break;
            }
        }
        // 角点2
        b_ = line[3] - k_ * line[2];
        for (int m = line[2]; m < line[2] + 40; m++) {
            int n = k_ * m + b_;
            cv::Point pt1(m, n);
            cv::Point pt2(line[2], line[3]);
            if (calu_dis_2point(pt1, pt2) > d) {
                pts.push_back(pt1);
                pts.push_back(pt2);
                break;
            }
        }
    }
    return 1;
}
bool get_rect_points(cv::Vec6f & line,
                     cv::Mat & im,
                     int d,
                     int type,
                     std::vector<cv::Point> & pts)
{
    cv::Vec4f t;
    for (int i = 0; i < 4; ++i)
    {
        t[i] = line[i];
    }
    get_rect_points(t,
                    im,
                    d,
           type,
            pts);
}
double calu_points_num_inRect(std::vector<cv::Point> & pts,
                              cv::Mat & im, int type)
{
    bool debug = 1;
    if (pts.size() < 1)
    {
        if (debug)
        {
            std::cout << "calu_points_num_inRect(), rect 不存在" << std::endl;
        }
        return -1;
    }
    cv::Point verticesp[1][4];
    verticesp[0][0] = pts[0];
    verticesp[0][1] = pts[1];
    verticesp[0][2] = pts[2];
    verticesp[0][3] = pts[3];
    const cv::Point *ppt[1] = {verticesp[0]};
    int npt[] = {4};
    cv::Mat mask;
    if (type == 1)
    {
        mask = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1); //750, 298
    }
    else
    {
        mask = cv::Mat::zeros(cv::Size(750, 298), CV_8UC1); //750, 298
    }
    fillPoly(mask, ppt, npt, 1, 255);
    double sum_rect_top = cv::countNonZero(mask);
    cv::bitwise_and(mask, im, mask);
    double  sum_255_top = cv::countNonZero(mask);

    return sum_255_top/sum_rect_top;

}


bool classify_hough_lines(std::vector<cv::Vec4f> & plines,
                          std::vector<cv::Vec4f> & plines_ipm,
                          cv::Mat & img_ps_mask_ipm,
                          int search_len,
                          double black_t, //0值比例阈值
                          std::vector<cv::Vec4f> & pos_lines,
                          std::vector<cv::Vec4f> & neg_lines)
{
    bool debug = 0;
    cv::Mat img_ps_mask_ipm3c;
    if (debug)
    {
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
    }

    double r255_1 = 0; double r255_2 = 0;
    for (int j = 0; j < plines.size(); ++j)
    {
        cv::Vec4f lin = plines[j];
        cv::Vec4f line = plines_ipm[j];
        cv::Vec4f line_ipm = plines_ipm[j];
        double alpha = calu_alpha_line(line_ipm, 0);

        if (alpha >= 45 && alpha <= 135) //左右矩形
        {
            std::cout << "alpha= " << alpha << std::endl;
            cv::Point2f midp((line[0]+line[2])/2,(line[1]+line[3])/2);
            if (1)
            {
                cv::circle(img_ps_mask_ipm3c, midp,2 ,cv::Scalar(0,0,255),-1,8,0);
                cv::imshow("img_ps_mask_ipm3c", img_ps_mask_ipm3c);
                cv::waitKey(0);
            }

            r255_1 = calu_255r_in_v_line(line,
                                img_ps_mask_ipm,
                                midp,
                                10, 3);
            r255_2 = calu_255r_in_v_line(line,
                                         img_ps_mask_ipm,
                                         midp,
                                         10, 4);
            std::cout << "r255_1= " << r255_1 << std::endl;
            std::cout << "r255_2= " << r255_2 << std::endl;
            bool is_pos = 0;
            if (r255_1 < 0.5 && r255_2 > 0.5)
            {
                //pos候选，需要进一步确定(U型引导线会出现伪pos或者neg)
                r255_1 = calu_255r_in_v_line(line,
                                             img_ps_mask_ipm,
                                             midp,
                                             40, 3);
                std::cout << "pos 确认 r255_1= " << r255_1 << std::endl;
                if (r255_1 < 0.2)  //pos neg的右矩形 255占比应该极小
                {
                    is_pos = 1;
                    pos_lines.push_back(lin);
                }
            }
            if (r255_1 > 0.5 && r255_2 < 0.5) //neg候选
            {
                r255_2 = calu_255r_in_v_line(line,
                                             img_ps_mask_ipm,
                                             midp,
                                             40, 4); //20cm
                std::cout << "neg 确认 r255_2= " << r255_2 << std::endl;
                if(r255_2 < 0.2)
                {
                    is_pos = 0;
                    neg_lines.push_back(lin);
                }
            }
            if(debug)
            {
                std::vector<cv::Point> pts;
                for (int i = 0; i < pts.size(); ++i)
                {
                    cv::circle(img_ps_mask_ipm3c, pts[i], 3, cv::Scalar(255, 0, 0), -1, 8, 0);
                    cv::imshow("img_ps_mask_ipm3c", img_ps_mask_ipm3c);
                }
                std::cout << "***************" << std::endl;
                std::cout << "r255_1= " << r255_1 << std::endl;
                std::cout << "r255_2= " << r255_2 << std::endl;

                for (int i = 0; i < pos_lines.size(); ++i)
                {
                    cv::line(img_ps_mask_ipm3c, cv::Point(pos_lines[i][0], pos_lines[i][1]),
                             cv::Point(pos_lines[i][2], pos_lines[i][3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
                }
                for (int i = 0; i < neg_lines.size(); ++i)
                {
                    cv::line(img_ps_mask_ipm3c, cv::Point(neg_lines[i][0], neg_lines[i][1]),
                             cv::Point(neg_lines[i][2], neg_lines[i][3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
                }
                if(is_pos)
                {
                    std::cout << "pos line" << std::endl;
                }
                else
                {
                    std::cout << "neg line" << std::endl;
                }
            }
        }
        else //上下矩形
        {
            cv::Point2f midp((line[0]+line[2])/2,(line[1]+line[3])/2);
            r255_1 = calu_255r_in_v_line(line,
                                         img_ps_mask_ipm,
                                         midp,
                                         10, 1); //返回上方5像素直线, 255所占的比例。
            r255_2 = calu_255r_in_v_line(line,
                                         img_ps_mask_ipm,
                                         midp,
                                         10, 2);//返回下方5像素直线, 255所占的比例。
            if (r255_1 < 0.5 && r255_2 > 0.5)
            {
                //pos候选，需要进一步确定(U型引导线会出现伪pos或者neg)
                r255_1 = calu_255r_in_v_line(line,
                                             img_ps_mask_ipm,
                                             midp,
                                             40, 1); //返回上方15像素直线, 255所占的比例。
                if (r255_1 < 0.1)
                {
                    pos_lines.push_back(lin);
                }
            }
            if (r255_1 > 0.5 && r255_2 < 0.5) //neg候选
            {
                r255_2 = calu_255r_in_v_line(line,
                                             img_ps_mask_ipm,
                                             midp,
                                             40, 2); //返回上方15像素直线, 255所占的比例。
                if(r255_2 < 0.1)
                {
                    neg_lines.push_back(lin);
                }
            }
        }
    }
    return 1;
}


bool lines_lsm_fit(std::vector<cv::Vec4f> & lines, cv::Vec6f & line, cv::Mat & img_pos_s_lines)
{
   // polynomial_curve_fit(std::vector<cv::Point>& points, int rank, cv::Mat& coef);
    double alpha = calu_alpha_line(lines[0], 0) + 1e-10;
    //std::cout<<"alpha"<<alpha<<std::endl;

    if(alpha > 45 && alpha < 135) //端点由y轴值确定
    {
        std::vector<cv::Point> points;
        points.clear();

        cv::Point tp_s,tp_e;
        double miny = 1000;
        double maxy = 0;
        for(int i=0; i<lines.size();i++)
        {
            tp_s.x = lines[i][1];  tp_s.y = lines[i][0];
            tp_e.x = lines[i][3];  tp_e.y = lines[i][2];
            if(tp_s.x < miny)
            {
                miny = tp_s.x;
            }
            if(tp_e.x < miny)
            {
                miny = tp_e.x;
            }
            if(tp_s.x > maxy)
            {
                maxy = tp_s.x;
            }
            if(tp_e.x > maxy)
            {
                maxy = tp_e.x;
            }
            if (abs(tp_s.y - tp_e.y) == abs(tp_s.x - tp_e.x)) //数值稳定性
            {
                tp_s.x = tp_s.x + 1;
            }
            points.push_back(tp_s);
            points.push_back(tp_e);
        }
        cv::Mat coef = cv::Mat::zeros(2, 1, CV_64FC1);
        polynomial_curve_fit(points, 1, coef);
        //std::cout<<"miny"<<miny<<std::endl;
        //std::cout<<"maxy"<<maxy<<std::endl;
        if (miny < maxy)
        {
            line[3] = miny;
            line[2] = coef.at<double>(0,0) + coef.at<double>(1,0) * miny;
            line[1] = maxy;
            line[0] = coef.at<double>(0,0) + coef.at<double>(1,0) * maxy;
        }
        else
        {
            line[3] = maxy;
            line[2] = coef.at<double>(0,0) + coef.at<double>(1,0) * maxy;
            line[1] = miny;
            line[0] = coef.at<double>(0,0) + coef.at<double>(1,0) * miny;
        }
        line[4] = coef.at<double>(1,0); //k
        line[5] = coef.at<double>(0,0); //b
    }
    else
    { //端点由x值确定的情况
        std::vector<cv::Point> points;
        points.clear();
        cv::Point tp_s, tp_e;
        double minx = 1000;
        double maxx = 0;
        for(int i=0; i<lines.size();i++)
        {
            tp_s.x = lines[i][0];  tp_s.y = lines[i][1];
            tp_e.x = lines[i][2];  tp_e.y = lines[i][3];
            if(tp_s.x < minx)
            {
                minx = tp_s.x;
            }
            if(tp_e.x < minx)
            {
                minx = tp_e.x;
            }
            if(tp_s.x > maxx)
            {
                maxx = tp_s.x;
            }
            if(tp_e.x > maxx)
            {
                maxx = tp_e.x;
            }
            if (abs(tp_s.y - tp_e.y) == abs(tp_s.x - tp_e.x)) //数值稳定性
            {
                tp_s.x = tp_s.x + 1;
            }
            points.push_back(tp_s);
            points.push_back(tp_e);
        }
        //std::cout<<"minx"<<minx<<std::endl;
        //std::cout<<"maxx"<<maxx<<std::endl;

        cv::Mat coef = cv::Mat::zeros(2, 1, CV_64FC1);
        polynomial_curve_fit(points, 1, coef);
        double miny = coef.at<double>(0,0) + coef.at<double>(1,0) * minx;
        double maxy = coef.at<double>(0,0) + coef.at<double>(1,0) * maxx;
        if (miny < maxy)
        {
            line[0] = maxx;
            line[1] = maxy;

            line[2] = minx;
            line[3] = miny;
        }
        else
        {
            line[0] = minx;
            line[1] = miny;

            line[2] = maxx;
            line[3] = maxy;
        }
        line[4] = coef.at<double>(1,0); //k
        line[5] = coef.at<double>(0,0); //b

        //std::cout<<"point1[0]"<<line[0]<<std::endl;
        //std::cout<<"point1[1]"<<line[1]<<std::endl;
    }
    return true;
}

bool find_end_point(std::vector<cv::Vec6f> & pos_separating_lines,
                    cv::Mat & img_ps_mask_ipm)
{
    for(int i=0; i<pos_separating_lines.size(); i++)
    {
        cv::Point p_end(pos_separating_lines[i][2],pos_separating_lines[i][3]);
        int x0;
        int y0;
        while(1)
        {
            x0 = p_end.x;
            y0 = p_end.y;
            int left_0_sum = 0;
            int right_0_sum = 0;
            for(int x=x0-20; x<x0; x=x+1)
            {
                for (int y=y0; y<y0+20; y=y+1)
                {
                    //std::cout<<pos_separating_lines[i][3] << std::endl;
                    if (img_ps_mask_ipm.at<uchar>(y,x) == 0)
                    {
                        left_0_sum++;
                    }
                }
            }
            for(int x=x0; x<x0+20; x=x+1)
            {
                for (int y=y0; y<y0+20; y=y+1)
                {
                    //std::cout<<pos_separating_lines[i][3] << std::endl;
                    if (img_ps_mask_ipm.at<uchar>(y,x) == 0)
                    {
                        right_0_sum++;
                    }
                }
            }
            if(left_0_sum/400.0 > 0.98 && right_0_sum/400.0 > 0.98)
            {
                pos_separating_lines[i][2] = p_end.x;
                pos_separating_lines[i][3] = p_end.y;
                break;
            }
            else
            {
                p_end.y = p_end.y + 1;
                p_end.x = pos_separating_lines[i][4] * p_end.y + pos_separating_lines[i][5];
            }
        }

        cv::Point p_start(pos_separating_lines[i][0],pos_separating_lines[i][1]);
        while(1)
        {
            if(calu_dis_2point(p_start, p_end) > 200)
            {
                pos_separating_lines[i][0] = p_start.x;
                pos_separating_lines[i][1] = p_start.y;
                break;
            }
            else
            {
                p_start.y = p_start.y - 1;
                p_start.x = pos_separating_lines[i][4] * p_start.y + pos_separating_lines[i][5];
            }
        }
    }
    return 0;
}

int find_separating_line_end_point(std::vector<cv::Vec6f> & pos_separating_lines,
                                   cv::Mat & img_ps_mask_ipm,
                                   cv::Mat & img_ps_bgr_ipm,
                                   std::vector<cv::Vec6f> & pos_separating_lines_dst,
                                   std::vector<L_shape_type> & L_point)
{
    pos_separating_lines_dst.clear();
    L_point.clear();


    if (pos_separating_lines.size() < 1)
    {
        return -1; // 输入为空
    }
    for (int i = 0; i < pos_separating_lines.size(); ++i)
    {
        cv::Vec6f line = pos_separating_lines[i];
        double alpha = calu_alpha_line(line, 0) - 1e-10;

        if (alpha > 45 && alpha < 135) //左右搜索
        {

            std::cout << "alpha= " << alpha <<  std::endl;
            for (int j = 0; j < 6; ++j)
            {
                std::cout << "line[] = "<<j<<"=" << line[j]  <<  std::endl;
            }

            for(int y = line[1]; y < 300; y++)
            {
                double x =  line[4] * y + line[5]; // kx+b
                cv::Point2f pt(x,y);
                double r_left = calu_255r_in_v_line(line,
                                    img_ps_mask_ipm,
                                    pt,
                                    15,
                                    3);
                double r_right = calu_255r_in_v_line(line,
                                                    img_ps_mask_ipm,
                                                    pt,
                                                    15,
                                                    4);
                if (r_left < 0.05 && r_right < 0.05)
                {
                    line[0] = x; line[1] = y; //终端坐标
                    //判断分割线是否完整(只有完整的分割线才push_back)
                    int d_dis = 20; //线终端距离 边缘10cm以上才认为完整
                    double d_y = y + d_dis;
                    double d_x = line[4] * d_y + line[5];
                    uchar value = img_ps_bgr_ipm.at<cv::Vec3b>(y + d_dis, x)[0];
                    if (1)
                    {
                        pos_separating_lines_dst.push_back(line);
                        L_shape_type t_L; //右角点
                        t_L.alpha = alpha;
                        t_L.vertical_line = line;
                        t_L.point_L = cv::Point2f(x,y);
                        L_point.push_back(t_L);
                    }
                    break;
                }
            }
        }
        else if (alpha >=0 && alpha < 45)
        {
            for(int x = line[0]; x > 0; x--)
            {
                double y =  line[4] * x + line[5]; // kx+b
                cv::Point2f pt(x,y);
                double r_top = calu_255r_in_v_line(line,
                                                    img_ps_mask_ipm,
                                                    pt,
                                                    15,
                                                    1);
                double r_bottom = calu_255r_in_v_line(line,
                                                     img_ps_mask_ipm,
                                                     pt,
                                                     15,
                                                     2);
                if (r_top < 0.05 && r_bottom < 0.05)
                {
                    line[0] = x; line[1] = y;
                    pos_separating_lines_dst.push_back(line);
                    break;
                }
            }
        }
        else
        {
            for(int x = line[0]; x < 1000; x++)
            {
                double y =  line[4] * x + line[5]; // kx+b
                cv::Point2f pt(x,y);
                double r_top = calu_255r_in_v_line(line,
                                                   img_ps_mask_ipm,
                                                   pt,
                                                   15,
                                                   1);
                double r_bottom = calu_255r_in_v_line(line,
                                                      img_ps_mask_ipm,
                                                      pt,
                                                      15,
                                                      2);
                if (r_top < 0.05 && r_bottom < 0.05)
                {
                    line[0] = x; line[1] = y;
                    pos_separating_lines_dst.push_back(line);
                    break;
                }
            }
        }
    }

    return 0;
}

bool find_parking_space_in_posneg_separating_lines(
        std::vector<cv::Vec6f> & pos_separating_lines,
        std::vector<cv::Vec6f> & neg_separating_lines,
        now_ps_type & ps_all)
{
//    ps_all.ps.clear();
//
//    for(int i=0; i<neg_separating_lines.size(); i++)
//    {
//        for(int j = 0; j< pos_separating_lines.size(); j++)
//        {
//            double dis = calu_dis_2lines_m2(pos_separating_lines[j], neg_separating_lines[i]);
//            //std::cout << "dis = " << dis << std::endl;
//            double param = calu_para_2lines(pos_separating_lines[j], neg_separating_lines[i]);
//
//            //std::cout << "pos_separating_lines[j][0] = " << pos_separating_lines[j][0] << std::endl;
//            //std::cout << "neg_separating_lines[i][0] = " << neg_separating_lines[i][0] << std::endl;
//
//            if (pos_separating_lines[j][0] > neg_separating_lines[i][0]
//                && dis> 80 && dis < 130 ) //1.6m-2.6m之间
//            {
//                std::cout << "find one ps " <<  std::endl;
//                parking_space_type ps;
//                ps.line_left = neg_separating_lines[i];
//                ps.line_right = pos_separating_lines[j];
//                ps_all.ps.push_back(ps);
//                break;
//            }
//        }
//    }
    return true;
}

cv::Point2f calu_point_2lines(cv::Vec6f & line1, cv::Vec6f & line2)
{

}

bool find_stop_lines(std::vector<cv::Vec6f> & pos_separating_lines,
                     std::vector<cv::Vec6f> & neg_separating_lines,
                     int & stop_line_num,
                     std::vector<parking_space_line_type> & stop_line)
{
    stop_line.clear();
    for(int i=0; i< pos_separating_lines.size(); i++)
    {
        double alpha_pos = calu_alpha_line(pos_separating_lines[i], 0);
        if (alpha_pos > -1 && alpha_pos < 30 || alpha_pos > 150 && alpha_pos < 180)
        {
            for (int j = 0; j < neg_separating_lines.size(); ++j)
            {
                double alpha_neg = calu_alpha_line(neg_separating_lines[j], 0);
                if (alpha_neg > -1 && alpha_neg < 30 || alpha_neg > 150 && alpha_neg < 180)
                {
                    double dis = calu_dis_2lines_m2(pos_separating_lines[i],
                                                    neg_separating_lines[j]);

                    if (dis > 3 && dis < 9) //6--14com
                    {
                        //std::cout <<"dis"<<dis<< std::endl;
                        //找到一条停止线
                        parking_space_line_type t;
                        t.pos_line = pos_separating_lines[i];
                        t.neg_line = neg_separating_lines[j];
                        stop_line.push_back(t);
                        break;
                    }
                }
            }
        }
    }
    if (stop_line.size() > 0)
    {
        stop_line_num = stop_line.size();
    }
    else
    {
        stop_line_num = -1;
    }
    //std::cout <<"stop_line_num"<<stop_line_num<< std::endl;
    return 1;
}

cv::Point2f calu_cutpoint_2lines(cv::Vec6f & line1, cv::Vec6f & line2)
{
    double x1 = line1[0];
    double y1 = line1[1];
    double x2 = line1[2];
    double y2 = line1[3];

    double x3 = line2[0];
    double y3 = line2[1];
    double x4 = line2[2];
    double y4 = line2[3];

    double b1 = (y2-y1)*x1+(x1-x2)*y1;
    double b2 = (y4-y3)*x3+(x3-x4)*y3;
    double D= (x2-x1)*(y4-y3)-(x4-x3)*(y2-y1);
    double D1=b2*(x2-x1)-b1*(x4-x3);
    double D2=b2*(y2-y1)-b1*(y4-y3);

    return cv::Point2f(D1/(D+1e-20),D2/(D+1e-20));
}


bool find_Lshape_point(std::vector<cv::Vec6f> & pos_separating_lines,
                       std::vector<cv::Vec6f> & neg_separating_lines,
                       std::vector<parking_space_line_type> & stop_line,
                       double line_alpha_error,
                       std::vector<L_shape_type> & L_shapes)
{
    L_shapes.clear();
    //double line_alpha_error = 3; //角度容许误差
    for (int i = 0; i < stop_line.size(); i++)
    {
        //该条stop_line相交形成的正交点
        for(int j=0;j<pos_separating_lines.size();j++)
        {
            double stop_line_alpha = calu_alpha_line(stop_line[i].pos_line, 0);
            double v_line_alpha = calu_alpha_line(pos_separating_lines[j], 0);
            double alpha_L = 0;
            if (abs(stop_line_alpha-v_line_alpha) < 90)
            {
                alpha_L = abs(stop_line_alpha-v_line_alpha);
            }
            else
            {
                alpha_L = 180 - abs(stop_line_alpha-v_line_alpha);
            }

            if(alpha_L > 90-line_alpha_error && alpha_L < 90+line_alpha_error)
            {
                //找到一个垂直角点
                L_shape_type t;
                t.line_type = 1;
                t.alpha = alpha_L;
                t.point_type = 1;
                t.stop_line = stop_line[i];
                t.vertical_line = pos_separating_lines[j];
                t.point_L = calu_cutpoint_2lines(stop_line[i].pos_line, pos_separating_lines[j]); //计算交点坐标
                L_shapes.push_back(t);
            }
            else if (alpha_L > 54-line_alpha_error && alpha_L < 54+line_alpha_error)
            {

            }
            else if(alpha_L > 138-line_alpha_error && alpha_L < 138+line_alpha_error)
            {

            }
            else
            {

            }
        }

        for (int k = 0; k < neg_separating_lines.size(); ++k)
        {
            double stop_line_alpha = calu_alpha_line(stop_line[i].pos_line, 0);
            double v_line_alpha = calu_alpha_line(neg_separating_lines[k], 0);
            double alpha_L = 0;
            if (abs(stop_line_alpha-v_line_alpha) < 90)
            {
                alpha_L = abs(stop_line_alpha-v_line_alpha);
            }
            else
            {
                alpha_L = 180 - abs(stop_line_alpha-v_line_alpha);
            }

            if(alpha_L > 90-line_alpha_error && alpha_L < 90+line_alpha_error)
            {
                //找到一个垂直角点
                L_shape_type t;
                t.line_type = 1;
                t.alpha = alpha_L;
                t.point_type = 2;
                t.stop_line = stop_line[i];
                t.vertical_line = neg_separating_lines[k];
                t.point_L = calu_cutpoint_2lines(stop_line[i].pos_line, neg_separating_lines[k]); //计算交点坐标
                L_shapes.push_back(t);
            }
            else if (alpha_L > 54-line_alpha_error && alpha_L < 54+line_alpha_error)
            {

            }
            else if(alpha_L > 138-line_alpha_error && alpha_L < 138+line_alpha_error)
            {

            }
            else
            {

            }

        }
    }
    return 1;
}

bool find_Lshape_point(std::vector<cv::Vec6f> & pos_separating_lines,
                       std::vector<cv::Vec6f> & neg_separating_lines,
                       cv::Mat & img_ps_mask_ipm,
                       cv::Mat & img_ps_bgr_ipm,
                       std::vector<L_shape_type> & L_shapes)
{
    L_shapes.clear();
    if(pos_separating_lines.size() > 0)
    {
        find_end_point(pos_separating_lines,img_ps_mask_ipm);
        for (int i = 0; i < pos_separating_lines.size(); ++i)
        {
            double alpha = calu_alpha_line(pos_separating_lines[i], 0);
            if (alpha < 20 || alpha > 160)
            {
                continue;
            }
            L_shape_type t;
            t.point_L = cv::Point2f(pos_separating_lines[i][2], pos_separating_lines[i][3]); //point pos
            t.point_type = 1; //1=pos line end point; 2=neg line end point
            t.vertical_line = pos_separating_lines[i];
            t.alpha = calu_alpha_line(pos_separating_lines[i], 0); //计算分割线的夹角
            //判断该角点是否满足区域条件
            double y = pos_separating_lines[i][3] + 10;
            double x = pos_separating_lines[i][4] * y + pos_separating_lines[i][5];
            //std::cout <<"y"<< y <<"x"<< x << std::endl;
            //std::cout << "img_ps_bgr_ipm.at<uchar>(y,x)" << img_ps_bgr_ipm.at<cv::Vec3b>(y,x) << std::endl;
            //如果满足在区域内的条件才push_back
            if (img_ps_bgr_ipm.at<cv::Vec3b>(y,x)[0] != 0)
            {
                L_shapes.push_back(t);
            }
        }
    }

    if(neg_separating_lines.size() > 0)
    {
        find_end_point(neg_separating_lines,img_ps_mask_ipm);
        for (int i = 0; i < neg_separating_lines.size(); ++i)
        {
            double alpha = calu_alpha_line(neg_separating_lines[i], 0);
            if (alpha < 20 || alpha > 160)
            {
                continue;
            }
            L_shape_type t;
            t.point_L = cv::Point2f(neg_separating_lines[i][2], neg_separating_lines[i][3]);
            t.point_type = 2; //neg line end point
            t.vertical_line = neg_separating_lines[i];
            t.alpha = calu_alpha_line(neg_separating_lines[i], 0); //计算分割线的夹角
            //判断该角点是否满足区域条件
            double y = neg_separating_lines[i][3] + 5;
            double x = neg_separating_lines[i][4] * y + neg_separating_lines[i][5];
            //std::cout <<"y"<< y <<"x"<< x << std::endl;
            //std::cout << "img_ps_bgr_ipm.at<uchar>(y,x)" << img_ps_bgr_ipm.at<cv::Vec3b>(y,x) << std::endl;
            //如果满足在区域内的条件才push_back
            if (img_ps_bgr_ipm.at<cv::Vec3b>(y,x)[0] != 0)
            {
                L_shapes.push_back(t);
            }
        }
    }
    return 1;
}

bool find_ps_in_L_shape_points(std::vector<L_shape_type> & L_shapes,
                               now_ps_type & ps_all)
{
    ps_all.ps.clear();
    for(int i=0; i<L_shapes.size(); i++)
    {
        if (L_shapes[i].point_type == 1) //正角点=pos 分割线与停止线的交点, 右入口角点
        {
            for (int j = 0; j < L_shapes.size(); ++j)
            {
                if(L_shapes[j].point_type == 2)
                {
                    double dis = calu_dis_2point2f(L_shapes[j].point_L, L_shapes[i].point_L);
                    double param = calu_para_2lines(L_shapes[j].vertical_line, L_shapes[i].vertical_line);

                    if(dis > 100 && dis < 130  //车位宽度约束
                            && param < 5       //分割线平行度约束
                            && L_shapes[i].point_L.x >L_shapes[j].point_L.x  //角点相对位置约束
                            )
                    {
                        //std::cout << "dis = " << dis << std::endl;
                        //std::cout << "param = " << param << std::endl;
                        parking_space_type t;
                        t.right_point = L_shapes[i];
                        t.left_point = L_shapes[j];
                        ps_all.ps.push_back(t);
                        break;
                    }
                }

            }
        }
    }
    if (ps_all.ps.size() > 0)
    {
        ps_all.is_has_ps = 1;
    }

}

int find_hough_lines_set(cv::Mat & img_ps_mask,
                         std::vector<cv::Vec4f> & hough_lines)
{
    //输入二值化图，返回直线集
    //ROI参数设置
    //cv::Mat img_ps_mask;
    //std::vector<cv::Vec4f> hough_lines;

    int roi_x = 0;
    int roi_y = 0;
    int roi_w = img_ps_mask.cols;
    int roi_h = img_ps_mask.rows;

    cv::Mat timg = img_ps_mask.clone();
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;             //原始轮廓
    cv::findContours(timg(cv::Rect(roi_x,roi_y,roi_w,roi_h)), contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE );

    //hough变换图像
    int hough_points=0;
    cv::Mat img_t = cv::Mat::zeros(img_ps_mask.size(), CV_8UC1);
    for (int i=0;i<contours.size();i++)
    {
        for(int j=0; j<contours[i].size();j++)
        {
            img_t.at<uchar>(contours[i][j].y, contours[i][j].x + roi_x) = 255;
            hough_points++;
        }
    }

    //hough transform
    hough_lines.clear();

    cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
    double start = static_cast<double>(cvGetTickCount());
    cv::HoughLinesP(img_t(cv::Rect(roi_x,roi_y,roi_w,roi_h)), hough_lines, 1, CV_PI / 180,
                    30,  // 判断直线的点数阈值
                    10,   //线段长度阈值
                    10      //线段上最近两点之间的阈值
    );
    double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    std::cout << "1->hough 边缘检测耗时:" << time/1000<<"ms"<<std::endl;

    for(int i=0; i< hough_lines.size(); i++)
    {
        hough_lines[i][0] = hough_lines[i][0] + roi_x;
        hough_lines[i][2] = hough_lines[i][2] + roi_x;
    }

    return hough_points; //the number of points of hough transform
}

int find_hough_lines_set(cv::Mat & img_ps_mask,
                         cv::Mat & ipm_mat,
                         std::vector<cv::Vec4f> & hough_lines)
{




}

int  perspective_transform_lines(std::vector<cv::Vec4f> &src,
                                 std::vector<cv::Vec4f> &dst,
                                 cv::Mat H)
{
    int line_num = src.size();
    if (line_num > 0)
    {
        std::vector<cv::Point2f>  p2f_src;
        std::vector<cv::Point2f>  p2f_dst;
        for (int i = 0; i < line_num ; ++i)
        {
            p2f_src.push_back(cv::Point2f(src[i][0], src[i][1]));
            p2f_src.push_back(cv::Point2f(src[i][2], src[i][3]));
        }

        cv::perspectiveTransform(p2f_src, p2f_dst, H);

        dst.clear();
        for (int j = 0; j < line_num; ++j)
        {
            cv::Vec4f line_t;
            double x1,y1,x2,y2;

            x1 = p2f_dst[j*2].x;
            y1 = p2f_dst[j*2].y;
            x2 = p2f_dst[j*2 + 1].x;
            y2 = p2f_dst[j*2 + 1].y;

            if (y1 > y2)
            {
                dst.push_back(cv::Vec4f(x1,y1,x2,y2));
            }
            else if(abs(y1 < y2) < 1e-5)
            {
                if (x1 < x2)
                {
                    dst.push_back(cv::Vec4f(x1,y1,x2,y2));
                } else
                {
                    dst.push_back(cv::Vec4f(x2,y2,x1,y1));
                }
            }
            else
            {
                dst.push_back(cv::Vec4f(x2,y2,x1,y1));
            }
        }
        return 1;
    }
    else
    {
        dst.clear();
        return -1; //输入线段数目为0
    }

}

double calu_255r_in_v_line(cv::Vec6f & line,
                         cv::Mat & im,
                         cv::Point2f &pt,
                         int d,
                         int type)
{
//    if (~ (type == 1 || type == 2 || type == 3 || type == 4))
//    {
//        return -1;
//    }

    double k = (line[3] - line[1]) / (line[2] - line[0] + 1e-15);
    double k_ = -1 / (k + 1e-15);
    double b_ = pt.y - k_ * pt.x; //y - kx
    double sum_0=0;
    double sum_255=0;
    if (type == 3)
    {
        for (int x = pt.x; x > pt.x - 40; x--)
        {
            double y = k_ * x + b_;
            cv::Point2f endp(x,y);
            //std:: cout << "endp" << endp << std::endl;
            //std:: cout << "calu_dis_2point2f(pt, endp)" << calu_dis_2point2f(pt, endp) << std::endl;
            if (calu_dis_2point2f(pt, endp)  < d) //搜索范围内
            {
                if (im.at<uchar>(y,x) == 0)
                {
                    sum_0 ++;
                }
                else
                {
                    sum_255 ++;
                }
            }
            else
            {
                break;
            }
        }
        return sum_255 / (sum_0 + sum_255);
    }
    else if (type == 4)
    {
        for( int x = pt.x; x < pt.x + 40; x++)
        {
            double y = k_ * x + b_;
            cv::Point2f endp(x,y);
            if(calu_dis_2point2f(pt, endp) < d)
            {
                if (im.at<uchar>(y,x) == 0)
                {
                    sum_0++;
                }
                else
                {
                    sum_255++;
                }
            }
            else
            {break;}
        }
        return sum_255 / (sum_0 + sum_255);
    }
    else if (type == 1)
    {
        for( int y = pt.y; y > pt.y - 40; y--)
        {
            double x = (y - b_)/(k_+1e-15);
            cv::Point2f endp(x,y);
            if(calu_dis_2point2f(pt, endp) < d)
            {
                if (im.at<uchar>(y,x) == 0)
                {
                    sum_0++;
                }
                else
                {
                    sum_255++;
                }
            }
            else
            {break;}
        }
        return sum_255 / (sum_0 + sum_255);
    } else if (type == 2)
    {
        for( int y = pt.y; y < pt.y + 40; y++)
        {
            double x = (y - b_)/(k_+1e-15);
            cv::Point2f endp(x,y);
            if(calu_dis_2point2f(pt, endp) < d)
            {
                if (im.at<uchar>(y,x) == 0)
                {
                    sum_0++;
                }
                else
                {
                    sum_255++;
                }
            }
            else
            {break;}
        }
        return sum_255 / (sum_0 + sum_255);
    }
}

double calu_255r_in_v_line(cv::Vec4f & line,
                           cv::Mat & im,
                           cv::Point2f &pt,
                           int d,
                           int type)
{
    cv::Vec6f tline;
    for (int i = 0; i < 4; ++i) {
        tline[i] = line[i];
    }
    tline[4]=tline[5]=0;

    return calu_255r_in_v_line(tline,
                               im,
                               pt,
                               d, type);
}

double classify_h_parking_space(parking_space_type & tps,
                                cv::Mat & mask)
{
    std::cout << "tps.left_point.alpha= " << tps.left_point.alpha << std::endl;
    std::cout << "tps.right_point.alpha= " << tps.right_point.alpha << std::endl;

    if (tps.left_point.alpha >= 45 && tps.left_point.alpha <=135
        && tps.right_point.alpha >= 45 && tps.right_point.alpha <=135) {

        cv::Point2f pstart = tps.left_point.point_L;
        //45--135
        for (int y = pstart.y; y > 0; y--) {
            double x = tps.left_point.vertical_line[4] * y +
                       tps.left_point.vertical_line[5];
            cv::Point2f pend(x, y);
            tps.left_point.point_end = pend;
            tps.left_point.vertical_line[2] = x;
            tps.left_point.vertical_line[3] = y;
            if (calu_dis_2point2f(pstart, pend) > 150) {
                break;
            }
        }

        pstart = tps.right_point.point_L;
        //45--135
        for (int y = pstart.y; y > 0; y--) {
            double x = tps.right_point.vertical_line[4] * y +
                       tps.right_point.vertical_line[5];
            cv::Point2f pend(x, y);
            tps.right_point.point_end = pend;
            tps.right_point.vertical_line[2] = x;
            tps.right_point.vertical_line[3] = y;
            if (calu_dis_2point2f(pstart, pend) > 150) {
                break;
            }
        }

        std::vector<cv::Point> pts_left, pts_right;
        get_rect_points(tps.left_point.vertical_line,
                        mask,
                        5, 3, pts_left);
        get_rect_points(tps.right_point.vertical_line,
                        mask,
                        5, 4, pts_right);



        double left_255 = calu_points_num_inRect(pts_left,
                                                 mask, 2);
        double right_255 = calu_points_num_inRect(pts_right,
                                                  mask, 2);
        std::cout << "left_255= " << left_255 << std::endl;
        std::cout << "right_255= " << right_255 << std::endl;
        if (left_255 > 0.7 && right_255 > 0.7)
            return (left_255 + right_255) / 2.0;
        else
            return -1;
    }
    else
    {
        return -1;
    }
}










