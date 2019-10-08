#include "tools.h"
#include "detect_parking_space.h"
#include "detect_line_space.h"

std::string intrinsicFileName_left;
std::string distortionFileName_left;
std::string perspectiveFileName_left;
std::string extrinsicFileName_left;

std::string intrinsicFileName_rear;
std::string distortionFileName_rear;
std::string perspectiveFileName_rear;
std::string extrinsicFileName_rear;


const double car_len = 500.5;       // Vehicle length
const double car_weight = 193.2;    //Vehicle width
const double car_zhouju = 187.5;    // Vehicle wheelbase
const double view_dis = 800.0;      //view distance 800cm


int main() {
    int camera_type_id = 0;         //定义相机类型1 锐尔威视，  0森云
    int is_test = 1;                //1--测评   0--检测
    int seq_id = 2;                 //测评时序列

    std::string img_base_dir;
    std::string param_base_dir = "/home/lx/data/suround_view_src_data/calibration";
    std::string camera_type;
    std::string img_distorted_dir_base;
    std::string img_undistorted_dir_base;
    std::string img_pred_dir_base;

    if (camera_type_id == 1)
    {
       camera_type = "weishi";
    }
    else
    {
        camera_type = "senyun";
    }

    if(is_test)
    {
        img_base_dir = "/home/lx/data/ceping_data";
        img_distorted_dir_base = img_base_dir + "/" + camera_type + "/seq" + std::to_string(seq_id) + "/distorted_images/";
        img_undistorted_dir_base = img_base_dir + "/" + camera_type + "/seq" + std::to_string(seq_id) + "/undistorted_images/";
        img_pred_dir_base = img_base_dir + "/" + camera_type + "/seq" + std::to_string(seq_id) + "/label_png_pred/";
    }
    else
    {
        //img_base_dir = "/home/lx/data/ceping_data";
    }

    intrinsicFileName_left = param_base_dir + "/" + camera_type + "/fish4/intrinsic.txt";
    distortionFileName_left = param_base_dir + "/" + camera_type + "/fish4/distortion.txt";
    perspectiveFileName_left = param_base_dir + "/" + camera_type + "/fish4/perspectiveTransformCalibration.txt";
    extrinsicFileName_left = param_base_dir + "/" + camera_type + "/fish4/extrinsic.txt";

    intrinsicFileName_rear = param_base_dir + "/" + camera_type + "/fish2/intrinsic.txt";
    distortionFileName_rear = param_base_dir + "/" + camera_type + "/fish2/distortion.txt";
    perspectiveFileName_rear = param_base_dir + "/" + camera_type + "/fish2/perspectiveTransformCalibration.txt";
    extrinsicFileName_rear = param_base_dir + "/" + camera_type + "/fish2/extrinsic.txt";

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

    std::string img_distorted_dir;
    std::string img_undistorted_dir;
    std::string img_pred_dir;

    //all img
    cv::Mat img_distorted;   //畸变图
    cv::Mat img_undistorted;  //去畸变图
    cv::Mat imgrear_distorted;   //畸变图
    cv::Mat imgrear_undistorted;  //去畸变图
    cv::Mat img_ipm;
    cv::Mat img_pred;   //预测结果，灰度图
    cv::Mat img_linep; //引导线
    cv::Mat img_linep_ipm;

    int now_n_img = 1;
    int now_frame_count = 1;
    parking_space  ps_search; //实例化车位线对象
    struct dirent **dirp;
    int n_file = scandir(img_pred_dir_base.c_str(), &dirp, 0, alphasort); //预测png图像路径
    if (n_file < 0)
    {
        std::cout << "no file" << std::endl;
    }
    else
    {
        int index = 2; //0 1 分别为 . .. dir
        while(index < n_file)
        {
            std::string tname = dirp[index]->d_name;
            free(dirp[index]);
            index++;
            std::vector<std::string> names = string_split(tname, ".");
            std::cout<<"img name now propressed is = " << tname <<std::endl;
            std::cout<<"img name now propressed is = " << names[0] <<std::endl;
            std::cout<<"img name now propressed is = " << names[1] <<std::endl;

            img_distorted_dir = img_distorted_dir_base + names[0] + ".jpg";
            img_undistorted_dir = img_undistorted_dir_base + names[0] + ".jpg";
            img_pred_dir = img_pred_dir_base + tname;

            img_distorted = cv::imread(img_distorted_dir);
            img_undistorted = cv::imread(img_undistorted_dir);
            img_pred = cv::imread(img_pred_dir,cv::IMREAD_GRAYSCALE);

            img_linep = img_pred.clone();   //get_mask_img(img_pred,img_linep, 4);
            get_mask_img(img_pred,img_linep, 4); //6
            //IPM
            img_undistort2topview(inter_params_left, distortion_left,
                                  rvecs_left, tvecs_left,"left",img_undistorted, img_ipm);
            img_undistort2topview(inter_params_left, distortion_left,
                                  rvecs_left, tvecs_left,"left",img_linep, img_linep_ipm);

            //角点检测
//            std::vector<cv::KeyPoint>detectKeyPoint;
////            //ORB
//            auto orb_detector = cv::ORB::create(10,1.2f,2);
//            //(int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31,
//             //       int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31, int fastThreshold=20);
//            double start = (double)cvGetTickCount();
//            orb_detector->detect(img_linep_ipm, detectKeyPoint);
//            double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
//            std::cout << "ORB :" << time/1000<<"ms"<<std::endl;

            //FAST
            //cv::threshold(img_linep_ipm, img_linep_ipm, 200, 255, cv::THRESH_BINARY);
            //double start = (double)cvGetTickCount();
            //cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(200,cv::FastFeatureDetector::TYPE_9_16);
            //fast->detect(img_linep_ipm,detectKeyPoint);

            //double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
            //std::cout << "ORB :" << time/1000<<"ms"<<std::endl;

//            cv::Mat img_linep_ipm_3c;  cv::cvtColor(img_linep_ipm, img_linep_ipm_3c, CV_GRAY2BGR);
//            cv::drawKeypoints(img_linep_ipm_3c,detectKeyPoint,img_linep_ipm_3c,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//            cv::imshow("img_linep_ipm", img_linep_ipm);
//            cv::imshow("img_linep_ipm_3c", img_linep_ipm_3c);
//            cv::waitKey(0);

            //车位线检测--车位搜索
            std::cout<<"img name : " << img_undistorted_dir << std::endl;

            ps_search.perspectiveFileName_left = perspectiveFileName_left;
            ps_search.img_ps_mask = img_linep; //引导线
            ps_search.img_ps_mask_ipm = img_linep_ipm;
            ps_search.img_ps_bgr = img_undistorted;
            ps_search.img_ps_bgr_ipm = img_ipm;
            if(ps_search.detect_closeed_ps())
            {
                break;
            }
            ps_search.show2();
        }
    }
    return 1;
}
