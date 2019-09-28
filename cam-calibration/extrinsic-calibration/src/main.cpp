#include<opencv2/opencv.hpp>
//#include <opencv.hpp>

#include<iostream>
#include<sstream>
#include "ReadParams.h"
 #include<time.h>


#define get_perspectiveMAt  0 //calibrate the perspective Mat
//标定外参数
#define calcute_extrisic_with_PNP_with_board 1
 //验证外参数
#define confirm_extrisic_with_calibrate 0  //手动选点
#define confirm_extrisic_withb_backproject 0  //相机坐标系点反投影


using namespace std;
using namespace cv;

cv::Size imageSize(640, 480);
// camera1 intrinsic and extrinsic param
std::vector<double> distortion;
cv::Mat intrinsic(cv::Size(3, 3), CV_64FC1);
cv::Mat perspectiveMat(cv::Size(3, 3), CV_64FC1);
cv::Mat shifPerspectiveMat(cv::Size(3, 3), CV_64FC1);
cv::Mat rotation_vectors(cv::Size(3, 1), CV_64FC1);
cv::Mat translation_vectors(cv::Size(3, 1), CV_64FC1);
cv::Mat mapx(imageSize, CV_32FC1);
cv::Mat mapy(imageSize, CV_32FC1);

std::vector<cv::Point> mousePoint;

// fish4 lateral camera parameters
const std::string distortionFileName = "/home/lx/data/suround_view_src_data/calibration/weishi/fish_test/distortion.txt";
const std::string intrinsicFileName = "/home/lx/data/suround_view_src_data/calibration/weishi/fish_test/intrinsic.txt";
const std::string perspectiveFileName = "/home/lx/data/suround_view_src_data/calibration/weishi/fish_test/perspectiveTransformCalibration.txt";
const std::string extrinsicFileName = "/home/lx/data/suround_view_src_data/calibration/weishi/fish_test/extrinsic.txt";
Mat org;
int n = 0;
vector<Point> capturePoint;
void on_mouse(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    Point pt;//坐标点;
    char coordinateName[16];

    if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取坐标，并在图像上该点处划圆
    {
        pt = Point(x, y);
        cout << x << " " << y << endl;
        capturePoint.push_back(pt);
        cout << capturePoint[n].x << " " << capturePoint[n].y << endl;
        cout << "n=" << n << endl;
        n++;
        circle(org, pt, 2, Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);//划圆
        sprintf(coordinateName, "(%d,%d)", x, y);
        putText(org, coordinateName, pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255), 1, 8);//在窗口上显示坐标
        if (n >= 1)
        {
            //imshow("org", org);
            cvDestroyAllWindows();
        }
    }
}


int main()
{

    cv::Mat R = Mat::eye(3, 3, CV_32F);
    intrinsic = readIntrinsicParams(intrinsicFileName);
    distortion = readDistortionParams(distortionFileName);
    readPerspectiveParams(perspectiveFileName, perspectiveMat, shifPerspectiveMat);
    readExtrinsic(extrinsicFileName, rotation_vectors, translation_vectors);
    cv::fisheye::initUndistortRectifyMap(intrinsic, distortion, R, intrinsic, imageSize, CV_32FC1, mapx, mapy);


#if (get_perspectiveMAt)
    {
        if (perspectiveTransCalibration())
            std::cout << "calibration success!" << std::endl;

    }
#endif
#if(confirm_extrisic_withb_backproject)
    {
        cv::VideoCapture cap(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

        cv::Mat frame;
        cv::Mat resize_img;
        float boardH = 20;
        std::vector<cv::Point3f> r;
        std::vector<cv::Point2f> imgPoints;


        if (!cap.isOpened())
        {
            std::cout << "摄像头读取失败" << std::endl;
            exit(EXIT_FAILURE);
        }
        for (int j = -1; j <= 5; j++)
            for (int i = -1; i <= 7; i++)
                r.push_back(cv::Point3f(boardH * i, boardH * j, 0));

        cv::fisheye::projectPoints(r,imgPoints,rotation_vectors, translation_vectors, intrinsic,distortion );
        while (cv::waitKey(30) != 27)
        {
            cap >> frame;
            resize_img = frame.clone();
//            cv::resize(frame,resize_img,imageSize);
            //cv::remap(resize_img, undistortionImage, mapx, mapy, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            if (!resize_img.empty())
            {
                for (int i = 0; i < imgPoints.size(); i++)
                {
                    cv::circle(resize_img, imgPoints[i], 3, cv::Scalar(0, 0, 255), -1);
                }
                imshow("CamPos", resize_img);
                cv::waitKey(1);
            }
        }


    }
#endif
#if (confirm_extrisic_with_calibrate)
    {
        cv::namedWindow("CamPos");
        cvSetMouseCallback("CamPos", on_mouse, NULL);
        cv::Mat frame;

        cv::VideoCapture cap(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        if (!cap.isOpened())
        {
            std::cout << "摄像头读取失败" << std::endl;
            exit(EXIT_FAILURE);
        }
        cv::Mat undistortionImage,resize_img;

        while (cv::waitKey(30) != 27)
        {
            cap >> frame;
//            cv::resize(frame,resize_img,imageSize);
            resize_img = frame.clone();
            cv::remap(resize_img, undistortionImage, mapx, mapy, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            if (!undistortionImage.empty())
            {
                if (cv::waitKey(1) == 'r')
                {
                    mousePoint.clear();
                    std::cout << "标记点已清除" << std::endl;
                }
                for (int i = 0; i < mousePoint.size(); i++)
                {
                    cv::circle(undistortionImage, mousePoint[i], 2, cv::Scalar(0, 255, 0), -1);
                }
                imshow("CamPos", undistortionImage);

                //imshow("CamPos", frame);
                cv::waitKey(5);
                if (mousePoint.size() == 1)
                {
                    std::cout << "图像坐标为:" << mousePoint << std::endl;

                    cv::Mat rrvec;
                    std::vector<cv::Point2f> objectPoints;
                    std::vector<cv::Point2f> imagePoints;
                    imagePoints.push_back(mousePoint[0]);

                    cv::Rodrigues(rotation_vectors, rrvec);
                    computeWorldcoordWithZ(imagePoints, objectPoints, rrvec, translation_vectors, intrinsic);

                    std::cout << "世界坐标为:" << objectPoints[0].x << "," << objectPoints[0].y << std::endl;
                    cv::waitKey(5000);
                    mousePoint.clear();
                }
            }
        }
    }
#endif

#if (calcute_extrisic_with_PNP_with_board)
    {
        cv::Size board_size = cv::Size(9, 6);
        std::vector<cv::Point2f> corners;
        float boardH = 2.3; //board :3.7cm
        float boardW = 2.3; //board :3.7cm
        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Point2f> imgPoints;
        std::vector<cv::Point3f> r;
        worldPoints.push_back(cv::Point3f(0,0,0));
        worldPoints.push_back(cv::Point3f((board_size.width-1)*boardW,0,0));
        worldPoints.push_back(cv::Point3f((board_size.width-1)*boardW,(board_size.height-1)*boardH,0));
        worldPoints.push_back(cv::Point3f((0,(board_size.height-1)*boardH,0)));

        bool calFlag = false;
        cv::Mat frame;
        cv::Mat resImg;
        cv::Mat undistortionImg;
        cv::VideoCapture cap(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        if (!cap.isOpened())
        {
            std::cout << "摄像头读取失败" << std::endl;
            exit(EXIT_FAILURE);
        }
        cv::Mat resize_img;

        while (cv::waitKey(0) != 27)
        {
            cap >> frame;
//            cv::resize(frame,resize_img,imageSize);
//            resImg = resize_img.clone();

            cv::remap(frame, undistortionImg, mapx, mapy, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
             if (!undistortionImg.empty())
            {
                cv::Mat tmp_img = undistortionImg.clone();
                for(int i=0; i<4; i++)
                {
                    namedWindow("tmp_img", 1);//定义一个org窗口
                    setMouseCallback("tmp_img", on_mouse, 0);//调用回调函数
                    imshow("tmp_img", tmp_img);
                    waitKey(0);
                }corners.clear();
                for(int i=0; i<4;i++)
                {
                    cout<<"capturePoint[i] "<<capturePoint[i]<<endl;
                    imagePoints.push_back(cv::Point2f(capturePoint[i]));
                }

                for (int j = 0; j < imagePoints.size(); j++)
                {
                        circle(undistortionImg, imagePoints[j], 3, Scalar(0, 0, 255), 2, 8, 0);
                }

                if(1)
                {
                    std::cout<<"******Extrinsic calibration...!*********"<<std::endl;
//                    imagePoints.push_back(corners[0]);
//                    imagePoints.push_back(corners[board_size.width-1]);
//                    imagePoints.push_back(corners[board_size.area()-1]);
//                    imagePoints.push_back(corners[board_size.width*(board_size.height-1)]);

                    std::cout<<"corner1 "<<imagePoints[0]<<std::endl;
                    std::cout<<"corner2 "<<imagePoints[1]<<std::endl;
                    std::cout<<"corner3 "<<imagePoints[2]<<std::endl;
                    std::cout<<"corner4 "<<imagePoints[3]<<std::endl;
                    // pnp solver calculate the extrinsic parameters: cv::SOLVEPNP_P3P,cv::SOLVEPNP_EPNP, cv::SOLVEPNP_ITERATIVE
                    //  cv::solvePnPRansac(worldPoints, corners, intrinsic, distortion, rotation_vectors, translation_vectors,false,100,8.0,0.99,cv::noArray(), cv::SOLVEPNP_P3P);
                    std::vector<double> distortion_zero(4,0);

                    cv::solvePnP(worldPoints, imagePoints, intrinsic, distortion_zero, rotation_vectors, translation_vectors, false, cv::SOLVEPNP_P3P);
                    std::ofstream fout_extrinsic(extrinsicFileName);

                    fout_extrinsic << rotation_vectors.at<double>(0,0) << std::endl;
                    fout_extrinsic << rotation_vectors.at<double>(0,1) << std::endl;
                    fout_extrinsic << rotation_vectors.at<double>(0,2) << std::endl;
                    fout_extrinsic << translation_vectors.at<double>(0,0) << std::endl;
                    fout_extrinsic << translation_vectors.at<double>(0,1) << std::endl;
                    fout_extrinsic << translation_vectors.at<double>(0,2) << std::endl;
                    std::cout<<"******Extrinsic calibration is done!*********"<<std::endl;
                    calFlag=true;
                    for (int j = 0; j < 6; j++)
                        for (int i = 0; i < 9; i++)
                            r.push_back(cv::Point3f(boardW * i, boardH * j, 0));

                    cv::fisheye::projectPoints(r,imgPoints,rotation_vectors, translation_vectors, intrinsic,distortion);
                    imagePoints.clear();
                    corners.clear();
                }

            }
            if(!frame.empty()&&calFlag)
            {
                for (int i = 0; i < imgPoints.size(); i++)
                {
                    cv::circle(frame, imgPoints[i], 3, cv::Scalar(0, 0, 255), -1);
                }
                imshow("世界坐标反投影结果", frame);
            }
            imshow("内角点检测结果", undistortionImg);
            cv::waitKey(1);
        }
    }
#endif

}
