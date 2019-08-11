#ifndef COMPUTE_WORLD_COOR_WITH_Z_H
#define COMPUTE_WORLD_COOR_WITH_Z_H 

//迭代求解方程 theta_d = theta ( 1 + D1*theta^2 + D2*theta^4 + D3*theta^6 + D4*theta^8 )
double solve_theta(double theta_d,cv::Mat & D)
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

//输入参数： 图像点的坐标 imagePoints，世界坐标系下点的z轴坐标 Z，世界坐标系转换到相机坐标系的 旋转矩阵 rvec，平移向量tvec，相机内参矩阵K，相机畸变系数D
//输出参数： 世界坐标系下点的坐标 objectPoints
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
    theta = solve_theta(theta_d,D);

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

#endif