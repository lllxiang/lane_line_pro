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
    int camera_type_id = 1;         //定义相机类型1 锐尔威视，  0森云
    int is_test = 0;                //1--测评   0--检测
    int seq_id = 4;                 //测评时序列
    std::string seq = "4_1_1";

    std::string img_base_dir;
    std::string param_base_dir = "/home/lx/data/suround_view_src_data/calibration";
    std::string camera_type;
    std::string img_distorted_dir_base;
    std::string img_undistorted_dir_base;
    std::string img_pred_dir_base;
    std::string ps_txt_file_dir;
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
        img_distorted_dir_base = img_base_dir + "/" + camera_type + "/"+seq+"/distorted_images/";
        img_undistorted_dir_base = img_base_dir + "/" + camera_type + "/"+seq+"/undistorted_images/";
        img_pred_dir_base = img_base_dir + "/" + camera_type + "/"+seq + "/label_png_pred/";
        ps_txt_file_dir = img_base_dir + "/" + camera_type + "/"+seq + "/pred_txt/pred_det.txt";
    }
    else
    {
        img_base_dir = "/home/lx/mnt/docker_data_4010/docker_data/surround_ipm7_test/mlednet/20191022060433/8left";
        img_undistorted_dir_base = img_base_dir + "/labeled_image_png/";
        img_pred_dir_base = img_base_dir + "/pred_image_png/";
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
    cv::Mat img_linep; // 分隔线
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
            //test IPM
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
            if(camera_type == "senyun")
            {
                get_mask_img(img_pred,img_linep, 3); //6
            } else{
                get_mask_img(img_pred,img_linep, 3); //6
            }
            cv::imshow("img_undistorted", img_undistorted);
            cv::imshow("img_linep", img_linep);
            cv::waitKey(0);

            //IPM
//            img_undistort2topview(inter_params_left, distortion_left,
//                                  rvecs_left, tvecs_left,"left","weishi",img_undistorted, img_ipm);
//            img_undistort2topview(inter_params_left, distortion_left,
//                                  rvecs_left, tvecs_left,"left","weishi",img_linep, img_linep_ipm);

//            //车位线检测--车位搜索
//            std::vector<std::string> img_names =  string_split(names[0], "_");
//            int frame_num = atoi(img_names[1].c_str());
//            std::cout << "img_name=" << img_names[1] << std::endl;
//            std::cout<<"img name : " << img_undistorted_dir << std::endl;
//            cv::threshold(img_linep_ipm, img_linep_ipm, 1, 255, cv::THRESH_BINARY);
            ps_search.frame_num_now = 1;
            ps_search.file_dir = ps_txt_file_dir;
            ps_search.perspectiveFileName_left = perspectiveFileName_left;
            //ps_search.img_ps_mask = img_linep; //引导线
            ps_search.img_ps_mask_ipm = img_linep;
//            ps_search.img_ps_bgr = img_undistorted;
            ps_search.img_ps_bgr_ipm = img_undistorted;
            if(ps_search.detect_ps_ipm())
            {
                break;
            }
            ps_search.show_ipm();

        }
    }
    return 1;
}
