#include "tools.h"
#include "detect_parking_space.h"
#include "detect_line_space.h"
#include "computeWorldcoordWithZ.h"

 std::string distortionFileName;
 std::string intrinsicFileName;
 std::string perspectiveFileName;
 std::string extrinsicFileName;

const double car_len = 500.5;    //自车长
const double car_weight = 193.2; //车宽
const double car_zhouju = 187.5;
const double view_dis = 500.0; //可视距离 800cm

//输入：方向标识，type,原图，
        //输出，IPM图
int ipm_trans(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp);

bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id);

//原图坐标->去畸变图坐标
bool src2dist(std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
    cv::Mat & mapx, cv::Mat & mapy);
bool read_cam_params(std::string &d);

int solve_super_rect(super_rect & sr);



int main()
{

    struct dirent *dirp;
    std::string img_labeled_dir = "/home/lx/data/surround_line14_test/lednet/6left/6left_img/";
    std::string img_pred_dir = "/home/lx/data/surround_line14_test/lednet/6left/6left_pred/";
    std::string img_labeled_all_dir, img_pred_all_dir;
    DIR* dir = opendir(img_labeled_dir.c_str());
    cv::Mat src_img,src_pred;
    int nImg = 251;
    while ((dirp = readdir(dir)) != NULL)
    {
        if (dirp->d_type == DT_REG)
        {
            std::string tname = dirp->d_name;
            int numof = tname.find_first_of(".");
            int numof_ = tname.find_first_of("_");
            //img_labeled_all_dir = img_labeled_dir + tname;
            // img_pred_all_dir = img_pred_dir + tname.substr(0,numof)+".png";
            //std::string direction = tname.substr(numof_ - 2 ,2);
            img_labeled_all_dir = img_labeled_dir + "im_" + std::to_string(nImg) + ".jpg";
            img_pred_all_dir = img_pred_dir + "im_" + std::to_string(nImg) + ".png";
            std::cout<<"当前处理的图片路径img_labeled_all_dir："<<img_labeled_all_dir << std::endl;
            std::cout<<"当前处理的图片路径img_pred_all_dir："<<img_pred_all_dir << std::endl;
            std::string direction = "ft";
            nImg++;
                    //读取相机参数
            read_cam_params(direction);
            //读取图片

            src_img = cv::imread(img_labeled_all_dir);
            src_pred = cv::imread(img_pred_all_dir,cv::IMREAD_GRAYSCALE);
            //提取目标类别
                    //目前逐个目标处理，后期可在分割mask图上直接进行一次findcontours, 减少计算量

            cv::Mat line_pred = cv::Mat::zeros(src_pred.size(),src_pred.type());
            //IPM
            cv::Mat src_img_ipm,line_pred_ipm;
            get_mask_img(src_pred,line_pred,6); //引导线 6
            int type = 1;
            ipm_trans(direction,type, line_pred, line_pred_ipm);
            ipm_trans(direction,type, src_img, src_img_ipm);


            //车位线检测--车位搜索
            parking_space  ps_search; //实例化车位线对象
            ps_search.img_ps_mask = line_pred; //引导线
            ps_search.img_ps_mask_ipm = line_pred_ipm;
            ps_search.img_ps_bgr = src_img;
            ps_search.img_ps_bgr_ipm = src_img_ipm;
            //ps_search.detect(); //执行检测操作。
            //ps_search.show();
            ps_search.detect_test();



//            //单白线检测
//            detect_s_w_line s_w_line_search;
//            s_w_line_search.img_ps_mask = line_pred; //引导线
//            s_w_line_search.img_ps_mask_ipm = line_pred_ipm;
//            s_w_line_search.img_ps_bgr = src_img;
//            s_w_line_search.img_ps_bgr_ipm = src_img_ipm;
//            if(s_w_line_search.detect())
//            {
//                s_w_line_search.show();
//            }


        }
    }

    std::string driection = "rear";
    std::string cam = "weishi";
    std::string fish = "fish2";
    int type = 1;   //0--畸变图   1 矫正图
    cv::Mat warp;
    cv::Mat img_jpg = cv::imread("/home/lx/data/surround_line14_test/ipm_test/weishi/img/"+driection+"/9rear_17.jpg");

    ipm_trans(driection,type, img_jpg, warp);


    cv::imshow("img_jpg", img_jpg);
    cv::imshow("warp", warp);
  cv::waitKey(0);
  return 0;
}

bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id)
{
    for(int h=0;h<src.rows; ++h)
    {
        for(int w=0;w<src.cols; ++w)
        {
            if (src.at<uchar>(h,w) == id )
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
bool src2dist(std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
              cv::Mat & mapx, cv::Mat & mapy)
{
    cv::Mat inv_mapx, inv_mapy,  inv_dst, inv_label;

    mapx.convertTo(mapx, CV_16UC1, 1, 0);
    mapy.convertTo(mapy, CV_16UC1, 1, 0);
    mapx.convertTo(mapx, CV_32FC1, 1, 0);
    mapy.convertTo(mapy, CV_32FC1, 1, 0);
    inv_mapx = cv::Mat::zeros(mapx.size(), CV_32FC1);
    inv_mapy = cv::Mat::zeros(mapy.size(), CV_32FC1);

    for(int i=0; i<mapx.rows;i++)
    {
        for(int j=0;j<mapx.cols;j++)
        {
            int x = int(mapx.at<float>(i,j));
            int y = int(mapy.at<float>(i,j));
            inv_mapx.at<float>(y,x) = j;
            inv_mapy.at<float>(y,x) = i;
        }
    }

    std::vector<cv::Point2f> dst_dist;
    for(int i=0;i<4;i++)
    {
        int x = src[i].x;
        int y = src[i].y;
        cv::Point2f tmp;
        tmp.x = inv_mapx.at<float>(y,x);
        tmp.y = inv_mapy.at<float>(y,x);
        dst.push_back(tmp);
    }
    return 1;
}

int ipm_trans(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp)
{
    read_cam_params(driection);

    double pp = 1;
    double pk = 0.9;
    cv::Mat img_dist;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat cameraMatrix2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs(cv::Size(1,4),CV_64F); //Size(w,h)
    std::vector<double> distortion;
    //std::cout<<"intrinsicFileName:"<<intrinsicFileName<<std::endl;
    cameraMatrix = readIntrinsicParams(intrinsicFileName);
    distortion = readDistortionParams(distortionFileName);
    cameraMatrix2.at<double>(0, 0) = cameraMatrix.at<double>(0, 0)*pk;
    cameraMatrix2.at<double>(0, 2) = cameraMatrix.at<double>(0, 2)*pp;
    cameraMatrix2.at<double>(1, 1) = cameraMatrix.at<double>(1, 1)*pk;
    cameraMatrix2.at<double>(1, 2) = cameraMatrix.at<double>(1, 2)*pp;
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    cv::Mat R; //旋转矩阵
    cv::Mat rvecs(cv::Size(1,3),CV_64F); //旋转向量
    cv::Mat tvecs(cv::Size(1,3),CV_64F); //平移矩阵
    readExtrinsic(extrinsicFileName, rvecs, tvecs);
    //std::cout<< rvecs << std::endl;
    cv::Rodrigues(rvecs, R);
    std::vector<cv::Point3f> wordPoints3D;  //俯视图 3D temp
    std::vector<cv::Point2f> wordPoints2D; //俯视图 2D
    std::vector<cv::Point2f> distImgPoint; //去畸变图上的点
    std::vector<cv::Point2f> srcImgPoint;   //畸变图上的点
    std::vector<double> Z; Z.push_back(0.0); Z.push_back(0.0); Z.push_back(0.0); Z.push_back(0.0);
    srcImgPoint.push_back(cv::Point2f(108.0,327.0));
    srcImgPoint.push_back(cv::Point2f(468.0,366.0));
    srcImgPoint.push_back(cv::Point2f(449.0,163.0));
    srcImgPoint.push_back(cv::Point2f(143.0,170.0));
    std::vector<cv::Point2f> srcImgPoint_calu_w = srcImgPoint; //srcImgPoint_calu_w 在computeWorldcoordWithZ
    //中会被改变值，因此准备一个备份

    //std::cout<< "旋转向量" << rvecs << std::endl;
    //std::cout<< "旋转矩阵" << R << std::endl;
    //std::cout<< "distCoeffs" << distCoeffs << std::endl;

    computeWorldcoordWithZ (wordPoints3D, srcImgPoint_calu_w, Z,R,
                            tvecs, cameraMatrix, distCoeffs);
    //确定 畸变图像点 与 俯视图 点对应关系
    for (int i=0; i<4; i++)
    {
        wordPoints2D.push_back(cv::Point2f(wordPoints3D[i].x,wordPoints3D[i].y));
        //std::cout<< "wordPoints2D:" << wordPoints2D[i] << std::endl;
    }
    //std::cout<< "distCoeffs" << distCoeffs.at<double>(3, 0) << std::endl;
    //std::cout<< "cameraMatrix" << cameraMatrix << std::endl;

    //图像去畸变
    cv::Size imageSize(640*pp, 480*pp);
    cv::Mat mapx(imageSize, CV_32FC1);
    cv::Mat mapy(imageSize, CV_32FC1);
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs,  cv::Mat(), cameraMatrix2, imageSize, CV_32FC1, mapx, mapy);
    cv::remap(img_jpg, img_dist, mapx, mapy, cv::INTER_NEAREST);
    //求去畸变图像对应的点坐标
    src2dist(srcImgPoint, distImgPoint, mapx, mapy);

    //根据去畸变图点坐标和俯视图点坐标求M
    cv::Size ipm_size;
    for (int i=0; i<4; i++)
    {
       // std::cout<<"distImgPoint in wh"<< distImgPoint[i] <<std::endl;
        //std::cout<<"wordPoints2D in xy"<< wordPoints2D[i] <<std::endl;
        if(driection == "ft")
        {
            //left
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + (car_len-car_zhouju)/2 + 106))/1;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis - 103))/1;
            ipm_size = cv::Size((view_dis*2 + car_len)/1,  (view_dis+car_weight/2)/1);
        }
        else if (driection == "ar")
        {
            //rear
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + 50))/2;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis + (car_len-car_zhouju)/2 - 210))/2;
            ipm_size = cv::Size((view_dis*2 + car_weight)/2,  (view_dis + car_len/2)/2);
        }
        //std::cout<<"wordPoints2D in wh"<< wordPoints2D[i] <<std::endl;
    }
    if (type)
    {//矫正图
        cv::Mat m = cv::getPerspectiveTransform(distImgPoint, wordPoints2D);
        std::cout<<"IPM变换矩阵m："<<m<<std::endl;
        cv::warpPerspective(img_jpg, warp, m, ipm_size);
    }
    else
    {
        cv::Mat m = cv::getPerspectiveTransform(distImgPoint, wordPoints2D);
        std::cout<<"IPM变换矩阵m："<<m<<std::endl;
        cv::warpPerspective(img_dist, warp, m, ipm_size);
        // cv::transform();

    }
    return 1;

}
bool read_cam_params(std::string &d)
{
    std::string fish;
    std::string cam = "weishi";
    //std::cout<<"d"<<d<<std::endl;
    if (d=="ar")
    {
        fish = "fish2";
    }
    else if (d == "ft")
    {
        fish = "fish4";
    }
    // 读取相机参数
    distortionFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/distortion.txt";
    intrinsicFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/intrinsic.txt";
    perspectiveFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/perspectiveTransformCalibration.txt";
    extrinsicFileName = "/home/lx/data/suround_view_src_data/calibration/"+cam+"/"+fish+"/extrinsic.txt";
    return 1;
}







