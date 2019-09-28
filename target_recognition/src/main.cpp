#include "tools.h"
#include "detect_parking_space.h"
#include "detect_line_space.h"


const std::string intrinsicFileName_left = "/home/lx/data/suround_view_src_data/calibration/weishi/fish4/intrinsic.txt";
const std::string distortionFileName_left = "/home/lx/data/suround_view_src_data/calibration/weishi/fish4/distortion.txt";
const std::string perspectiveFileName_left = "/home/lx/data/suround_view_src_data/calibration/weishi/fish4/perspectiveTransformCalibration.txt";
const std::string extrinsicFileName_left = "/home/lx/data/suround_view_src_data/calibration/weishi/fish4/extrinsic.txt";

const std::string intrinsicFileName_rear = "/home/lx/data/suround_view_src_data/calibration/weishi/fish2/intrinsic.txt";
const std::string distortionFileName_rear = "/home/lx/data/suround_view_src_data/calibration/weishi/fish2/distortion.txt";
const std::string perspectiveFileName_rear = "/home/lx/data/suround_view_src_data/calibration/weishi/fish2/perspectiveTransformCalibration.txt";
const std::string extrinsicFileName_rear = "/home/lx/data/suround_view_src_data/calibration/weishi/fish2/extrinsic.txt";

const double car_len = 500.5;    // Vehicle length
const double car_weight = 193.2; //Vehicle width
const double car_zhouju = 187.5; // Vehicle wheelbase
const double view_dis = 800.0; //view distance 800cm

int main() {
//    while(1)
//    {
//        //test fit fun
//        //polynomial_curve_fit(std::vector<cv::Point>& points, int rank, cv::Mat& coef);
//        //alpha 45 135之间：
//
//        std::vector<cv::Point> points;
//        points.push_back(cv::Point(200, 401));
//        points.push_back(cv::Point(300, 400));
//        points.push_back(cv::Point(400, 400));
//        cv::Mat coef = cv::Mat::zeros(2, 1, CV_64FC1);
//        polynomial_curve_fit(points, 1, coef);
//
//        cv::Mat im = cv::Mat::zeros(cv::Size(600,600), CV_8UC3);
//
//
//        for(int i=0; i<points.size();i++)
//        {
//            cv::circle(im, points[i],5,cv::Scalar(255,0,0),1,8,0);
//        }
//
//        for(int y=0; y<600; y++)
//        {
//            cv::Point ps;
//            ps.y = y;
//            ps.x = coef.at<double>(0,0) + coef.at<double>(1,0) * y;
//            cv::circle(im, ps,1,cv::Scalar(0,0,255),1,8,0);
//
//        }
//        cv::imshow("im", im);
//        cv::waitKey(0);
//    }
    //read left camera offline params
    cv::Mat inter_params_left(cv::Size(3, 3), CV_64FC1);
    cv::Mat inter_params_rear(cv::Size(3, 3), CV_64FC1);
    std::vector<double> distortion_left, distortion_rear;
    cv::Mat rvecs_left(cv::Size(1, 3), CV_64F); //Rotation vector
    cv::Mat tvecs_left(cv::Size(1, 3), CV_64F); //Translation vector
    cv::Mat rvecs_rear(cv::Size(1, 3), CV_64F);
    cv::Mat tvecs_rear(cv::Size(1, 3), CV_64F);

    inter_params_left = readIntrinsicParams(intrinsicFileName_left);
    distortion_left = readDistortionParams(distortionFileName_left);
    readExtrinsic(extrinsicFileName_left, rvecs_left, tvecs_left);

    inter_params_rear = readIntrinsicParams(intrinsicFileName_rear);
    distortion_rear = readDistortionParams(distortionFileName_rear);
    readExtrinsic(extrinsicFileName_rear, rvecs_rear, tvecs_rear);


    std::cout << "Read left camera params done!" << std::endl;

    //set img dir
//    std::string img_distorted_dir_base = "/home/lx/data/surround_line14_test/lednet/6left/6left_distort/";
//    std::string img_undistorted_dir_base = "/home/lx/data/surround_line14_test/lednet/6left/6left_img/";
//    std::string img_pred_dir_base = "/home/lx/data/surround_line14_test/lednet/6left/6left_pred/";
    int is_test = 0;

    std::string img_distorted_dir_base;
    std::string img_undistorted_dir_base;
    std::string img_pred_dir_base;
    if(is_test)
    {
        std::string base_dir = "/home/lx/data/surround_line14_test/lednet/8left";
        std::string seq_num = "/val_seq1";
        img_distorted_dir_base = base_dir + seq_num + "/images/";
        img_undistorted_dir_base = base_dir + seq_num + "/images/";
        img_pred_dir_base = base_dir + "/img_pred/";
    } else
    {
        img_distorted_dir_base = "/home/lx/data/surround_line14_test/lednet/6left/img_undistorted/";
        img_undistorted_dir_base = "/home/lx/data/surround_line14_test/lednet/6left/img_undistorted/";
        img_pred_dir_base = "/home/lx/data/surround_line14_test/lednet/6left/img_pred/";
    }


    std::string imgrear_distorted_dir_base = "/home/lx/data/surround_line14_test/lednet/10rear/img_distorted/";
    std::string imgrear_undistorted_dir_base = "/home/lx/data/surround_line14_test/lednet/10rear/img_undistorted/";
    std::string imgrear_pred_dir_base = "/home/lx/data/surround_line14_test/lednet/10rear/img_pred/";

    std::string img_distorted_dir;
    std::string img_undistorted_dir;
    std::string img_pred_dir;

    std::string imgrear_distorted_dir;
    std::string imgrear_undistorted_dir;
    std::string imgrear_pred_dir;

    //all img
    cv::Mat img_distorted;   //畸变图
    cv::Mat img_undistorted;  //去畸变图
    cv::Mat imgrear_distorted;   //畸变图
    cv::Mat imgrear_undistorted;  //去畸变图

    cv::Mat img_ipm;
    cv::Mat img_pred;   //预测结果，灰度图
    cv::Mat img_linep; //引导线
    cv::Mat img_linep_ipm;

//    struct dirent *dirp;
//    DIR *dir = opendir(img_undistorted_dir.c_str());
//    while ((dirp = readdir(dir)) != NULL)
//    {
//        if (dirp->d_type == DT_REG)
//        {
//            std::string tname = dirp->d_name;
//            std::cout<<"img name now propressed is = " << tname <<std::endl;
//        }
//    }
//    int now_n_img = 1;
//    while(now_n_img+1)
//    {
//        img_distorted_dir = img_distorted_dir_base + "im_" + std::to_string(now_n_img) + ".jpg";
//        img_undistorted_dir = img_undistorted_dir_base + "im_" + std::to_string(now_n_img) + ".jpg";
//
//        imgrear_distorted_dir = imgrear_distorted_dir_base + "im_" + std::to_string(now_n_img) + ".jpg";
//        imgrear_undistorted_dir = imgrear_undistorted_dir_base + "im_" + std::to_string(now_n_img) + ".jpg";
//        img_pred_dir = imgrear_pred_dir_base + "im_" + std::to_string(now_n_img) + ".png";
//
//        img_distorted = cv::imread(img_distorted_dir);
//        imgrear_distorted = cv::imread(imgrear_distorted_dir);
//
//        undistort_fish_img(inter_params_left,distortion_left, 1, 1,cv::Size(640*2, 480*2),img_distorted, img_undistorted);
//        undistort_fish_img(inter_params_rear,distortion_rear, 1, 1,cv::Size(640*2, 480*2),imgrear_distorted, imgrear_undistorted);
//
//        cv::resize(img_undistorted,img_undistorted,cv::Size(640,480));
//        cv::resize(imgrear_undistorted,imgrear_undistorted,cv::Size(640,480));
//
//        cv::Mat img_ipm, imgrear_ipm;
//        img_undistort2topview(inter_params_rear, distortion_rear,
//                              rvecs_rear, tvecs_rear,"rear",imgrear_undistorted, imgrear_ipm);
//
//        img_undistort2topview(inter_params_left, distortion_left,
//                              rvecs_left, tvecs_left,"left",img_undistorted, img_ipm);
//
//        cv::transpose(imgrear_ipm, imgrear_ipm);
//        cv::flip(imgrear_ipm,imgrear_ipm,0);
//
//        cv::imshow("img_undistorted", img_undistorted);
//        cv::imshow("img_ipm", img_ipm);
//
//        cv::imshow("imgrear_undistorted", imgrear_undistorted);
//        cv::imshow("imgrear_ipm", imgrear_ipm);
//
//        cv::waitKey(0);
//
//        now_n_img++;
//    }
    int now_n_img = 34;
    int now_frame_count = 1;

    parking_space  ps_search; //实例化车位线对象

    while(now_n_img)
    {
        img_distorted_dir = img_distorted_dir_base + "im_" + std::to_string(now_n_img) + ".jpg";
        img_undistorted_dir = img_undistorted_dir_base + "im_" + std::to_string(now_n_img) + ".jpg";
        img_pred_dir = img_pred_dir_base + "im_" + std::to_string(now_n_img) + ".png";

        img_distorted = cv::imread(img_distorted_dir);
        if(img_distorted.empty())
        {
            now_n_img++;
            continue;
        }
        img_undistorted = cv::imread(img_undistorted_dir);
        img_pred = cv::imread(img_pred_dir, cv::IMREAD_GRAYSCALE);
        img_linep = img_pred.clone();

        get_mask_img(img_pred,img_linep, 6);
        img_undistort2topview(inter_params_left, distortion_left,
                              rvecs_left, tvecs_left,"left",img_undistorted, img_ipm);
        img_undistort2topview(inter_params_left, distortion_left,
                              rvecs_left, tvecs_left,"left",img_linep, img_linep_ipm);

        //车位线检测--车位搜索
        std::cout<<"img name : " << img_undistorted_dir << std::endl;

        ps_search.frame_count = now_n_img;
        ps_search.now_frame_count = now_frame_count;
        ps_search.img_ps_mask = img_linep; //引导线
        ps_search.img_ps_mask_ipm = img_linep_ipm;
        ps_search.img_ps_bgr = img_undistorted;
        ps_search.img_ps_bgr_ipm = img_ipm;
//        ps_search.img_bgr_ipm_combined = img_ipm.clone();
//        ps_search.img_bgr_ipm_t = img_ipm.clone();
        //ps_search.detect(); //执行检测操作。
        //ps_search.show();
        if(ps_search.detect_test())
        {
            break;
        }
        ps_search.show2();
        now_frame_count+
//        cv::imshow("img_distorted", img_distorted);
//        cv::imshow("img_undistorted", img_undistorted);
//        cv::imshow("img_linep", img_linep);
//
//        cv::imshow("img_ipm", img_ipm);
//        cv::imshow("img_linep_ipm", img_linep_ipm);
//        cv::waitKey(0);
        now_n_img++;

    }
    return 1;



}






