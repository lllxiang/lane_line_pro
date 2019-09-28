#include<opencv2/opencv.hpp>
#include<fstream>
#include"ReadParams.h"

#define video_calibrate_perspective_Mat 1
#define camera_calibrate_perspective_Mat 0

/*****************************    coordinate system transform       *******************************/
void  computeWorldcoordWithZ(const std::vector<cv::Point2f>& imagePoints,
                                          std::vector<cv::Point2f>& objectPoints,
                                          const cv::Mat&  rvec,
                                          const cv::Mat&  tvec,
                                          const cv::Mat&  K)
{
    cv::Mat Ma = (cv::Mat_<double>(2, 2));
    cv::Mat Mb = (cv::Mat_<double>(2, 1));
    cv::Mat  c = (cv::Mat_<double>(2, 1));

    for (int i = 0; i < imagePoints.size(); ++i)
    {
        Ma.at<double>(0, 0) = K.at<double>(0, 0) * rvec.at<double>(0, 0) + K.at<double>(0, 2) * rvec.at<double>(2, 0) - rvec.at<double>(2, 0) * imagePoints[i].x;
        Ma.at<double>(0, 1) = K.at<double>(0, 0) * rvec.at<double>(0, 1) + K.at<double>(0, 2) * rvec.at<double>(2, 1) - rvec.at<double>(2, 1) * imagePoints[i].x;

        Ma.at<double>(1, 0) = K.at<double>(1, 1) * rvec.at<double>(1, 0) + K.at<double>(1, 2) * rvec.at<double>(2, 0) - rvec.at<double>(2, 0) * imagePoints[i].y;
        Ma.at<double>(1, 1) = K.at<double>(1, 1) * rvec.at<double>(1, 1) + K.at<double>(1, 2) * rvec.at<double>(2, 1) - rvec.at<double>(2, 1) * imagePoints[i].y;

        Mb.at<double>(0, 0) = tvec.at<double>(0, 2) * imagePoints[i].x - K.at<double>(0, 0) * tvec.at<double>(0, 0) - K.at<double>(0, 2) * tvec.at<double>(0, 2);//objectPoints[i].z ;
        Mb.at<double>(1, 0) = tvec.at<double>(0, 2) * imagePoints[i].y - K.at<double>(1, 1) * tvec.at<double>(0, 1) - K.at<double>(1, 2) * tvec.at<double>(0, 2);//objectPoints[i].z ;

        cv::solve(Ma, Mb, c, CV_SVD);
        objectPoints.push_back(cv::Point2f(c.at<double>(0, 0), c.at<double>(1, 0)));
    }
}
 bool findCorners(const cv::Mat img, cv::Size board_size, std::vector<cv::Point2f> &corners)
{
    cv::Mat imageGray;
    cv::cvtColor(img, imageGray, CV_RGB2GRAY);
    bool patternfound = cv::findChessboardCorners(img, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
        cv::CALIB_CB_FAST_CHECK);
    if (!patternfound)
        return false;
    else
    {
        cv::cornerSubPix(imageGray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        return true;
    }
}
std::vector<double> readDistortionParams(const std::string distortionFileName)
{
    std::vector<double> distortion;
    std::ifstream in;
    in.open(distortionFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open distortionFile!" << std::endl;
        exit(EXIT_FAILURE);
    }
    for (double i; in >> i;)
    {
        distortion.push_back(i);
    }
    in.close();
    return distortion;
}

cv::Mat readIntrinsicParams(const std::string intrinsicFileName)
{
    cv::Mat intrinsic(cv::Size(3, 3), CV_64FC1);
    std::ifstream in;
    in.open(intrinsicFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open intrinsicFile!" << std::endl;
        exit(EXIT_FAILURE);
    }

    intrinsic.setTo(0);
    in >> intrinsic.at<double>(0, 0);
    in >> intrinsic.at<double>(0, 2);
    in >> intrinsic.at<double>(1, 1);
    in >> intrinsic.at<double>(1, 2);
    intrinsic.at<double>(2, 2) = 1;
    in.close();
    return intrinsic;
}
void readExtrinsic(const std::string extrinsicFileName, cv::Mat &rotation_vectors, cv::Mat &translation_vectors)
{
    std::ifstream in;
    in.open(extrinsicFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open extrinsicFile!" << std::endl;
        exit(EXIT_FAILURE);
    }
    in >> rotation_vectors.at<double>(0, 0);
    in >> rotation_vectors.at<double>(0, 1);
    in >> rotation_vectors.at<double>(0, 2);

    in >> translation_vectors.at<double>(0, 0);
    in >> translation_vectors.at<double>(0, 1);
    in >> translation_vectors.at<double>(0, 2);
    in.close();

}

void readPerspectiveParams(const std::string perspectiveFileName, cv::Mat &perspectiveMat,cv::Mat &shifPerspectiveMat)
{
    //cv::Mat perspectiveMat(cv::Size(3, 3), CV_64FC1);
    std::ifstream in;

    in.open(perspectiveFileName, std::ios::in);
    if (!in.is_open())
    {
        std::cout << "fail to open perspectiveFile!" << std::endl;
        exit(EXIT_FAILURE);
    }
    in >> perspectiveMat.at<double>(0, 0);
    in >> perspectiveMat.at<double>(0, 1);
    in >> perspectiveMat.at<double>(0, 2);
    in >> perspectiveMat.at<double>(1, 0);
    in >> perspectiveMat.at<double>(1, 1);
    in >> perspectiveMat.at<double>(1, 2);
    in >> perspectiveMat.at<double>(2, 0);
    in >> perspectiveMat.at<double>(2, 1);
    in >> perspectiveMat.at<double>(2, 2);
    in >> shifPerspectiveMat.at<double>(0, 0);
    in >> shifPerspectiveMat.at<double>(0, 1);
    in >> shifPerspectiveMat.at<double>(0, 2);
    in >> shifPerspectiveMat.at<double>(1, 0);
    in >> shifPerspectiveMat.at<double>(1, 1);
    in >> shifPerspectiveMat.at<double>(1, 2);
    in >> shifPerspectiveMat.at<double>(2, 0);
    in >> shifPerspectiveMat.at<double>(2, 1);
    in >> shifPerspectiveMat.at<double>(2, 2);
    in.close();
 }


//void on_mouse(int event, int x, int y, int flag, void *param)
//{
//    if (event == CV_EVENT_LBUTTONDOWN)
//    {
//        mousePoint.push_back(cv::Point(x, y));
//        std::cout << "add 1 point " << std::endl;
//    }
//}

//display the mouse point
//void moving_mouse(int event, int x, int y, int flags, void* ustc)
//{
//    char charText[30];
//    cv::Mat tempImage;
//    tempImage = bird_img.clone();

//    if (event == CV_EVENT_MOUSEMOVE)
//    {
//        sprintf(charText, "(%d, %d)", x, y);
//        cv::Point pt = cv::Point(x, y);
//        putText(tempImage, charText, pt, cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
//        imshow("bird eye image", tempImage);
//    }
//}

bool perspectiveTransCalibration()
{
    std::vector<cv::Point> worldPoint;
    worldPoint.push_back(cv::Point(226, 400));
    worldPoint.push_back(cv::Point(414, 400));
    worldPoint.push_back(cv::Point(414, 40));
    worldPoint.push_back(cv::Point(226, 40));

    bool CALIBRATION_SUCCESS = false;

    cv::namedWindow("perspectiveTransCalibrationWindow");
    // cvSetMouseCallback("perspectiveTransCalibrationWindow", (CvMouseCallback)on_mouse, NULL);//



    const std::string PerspectiveTransformCalibration = "/home/baozhengfan/projection/intrinsic_param_calib_image_input/calibration/camera3/perspectiveTransformCalibration.txt";

    cv::Mat undistortionImage;
    cv::Mat frame;
    cv::Mat img;
    cv::Mat TransformMat;
    cv::Mat shiftTransformMat;
#if  (video_calibrate_perspective_Mat)
    cv::VideoCapture cap;
    cap.open("/home/baozhengfan/projection/catkin_ws/record_video/video201902011053.avi");
    if (!cap.isOpened())
    {
        std::cout << "fail to open the video" << std::endl;
        return -1;
    }
    double frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);

    for (;;)//(int i = 0; i < frame_count; i++)
    {
        cap >> frame;
        if (!frame.empty())
        {

            // press the 'space' to calibrate the transformMat
            while (cv::waitKey(5) != ' ')
            {
                if (cv::waitKey(5) == 'r')// press 'r' to clear all of the points
                {
                    mousePoint.clear();
                    std::cout << "all of the points have been cleared up" << std::endl;
                }
                cv::resize(frame,img,cv::Size(640,480));
                cv::remap(img, undistortionImage, mapx, mapy, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));


                for (int i = 0; i < mousePoint.size(); i++)
                {
                    cv::circle(undistortionImage, mousePoint[i], 2, cv::Scalar(0, 255, 0), -1);
                }
                cv::line(undistortionImage,cv::Point(0,undistortionImage.rows/2),cv::Point(undistortionImage.cols,undistortionImage.rows/2),cv::Scalar(0,255,0),1);
                cv::line(undistortionImage,cv::Point(undistortionImage.cols/2,0),cv::Point(undistortionImage.cols/2,undistortionImage.rows),cv::Scalar(0,255,0),1);

                cv::imshow("perspectiveTransCalibrationWindow", undistortionImage);
                 cv::waitKey(1);

                if (mousePoint.size() == 4)
                {
                    TransformMat = findHomography(mousePoint, worldPoint, CV_RANSAC);
                    shiftTransformMat = findHomography(worldPoint, mousePoint, CV_RANSAC);


                    cv::Mat PerspectiveTransformImage;
                    cv::warpPerspective(undistortionImage, PerspectiveTransformImage, TransformMat, undistortionImage.size(), CV_INTER_CUBIC);

                    cv::imshow("PerspectiveTransformImage", PerspectiveTransformImage);
                    cv::waitKey(5000);
                    cv::destroyWindow("PerspectiveTransformImage");

                    std::ofstream out(PerspectiveTransformCalibration);

                    out << TransformMat.at<double>(0, 0) << std::endl;
                    out << TransformMat.at<double>(0, 1) << std::endl;
                    out << TransformMat.at<double>(0, 2) << std::endl;
                    out << TransformMat.at<double>(1, 0) << std::endl;
                    out << TransformMat.at<double>(1, 1) << std::endl;
                    out << TransformMat.at<double>(1, 2) << std::endl;
                    out << TransformMat.at<double>(2, 0) << std::endl;
                    out << TransformMat.at<double>(2, 1) << std::endl;
                    out << TransformMat.at<double>(2, 2) << std::endl;
                    out << shiftTransformMat.at<double>(0, 0) << std::endl;
                    out << shiftTransformMat.at<double>(0, 1) << std::endl;
                    out << shiftTransformMat.at<double>(0, 2) << std::endl;
                    out << shiftTransformMat.at<double>(1, 0) << std::endl;
                    out << shiftTransformMat.at<double>(1, 1) << std::endl;
                    out << shiftTransformMat.at<double>(1, 2) << std::endl;
                    out << shiftTransformMat.at<double>(2, 0) << std::endl;
                    out << shiftTransformMat.at<double>(2, 1) << std::endl;
                    out << shiftTransformMat.at<double>(2, 2) << std::endl;
                    out.close();
                    mousePoint.clear();
                    CALIBRATION_SUCCESS = true;
                }

            }

        }

    }
    return CALIBRATION_SUCCESS;



#endif
#if (camera_calibrate_perspective_Mat)
    cv::VideoCapture cap(1);
    if (!cap.isOpened())
    {
        std::cout << "fail oppencv the camera?" << std::endl;
        exit(EXIT_FAILURE);
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    while (1)//
    {
        cap >> img;
        if (!img.empty())
        {
            if (cv::waitKey(1) == 'r')
            {
                mousePoint.clear();
                std::cout << "all of the points have been cleared up" << std::endl;
            }
           cv::resize(img,frame,cv::Size(640,480));
           cv::remap(frame, undistortionImage, mapx, mapy, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            for (int i = 0; i < mousePoint.size(); i++)
            {
                cv::circle(undistortionImage, mousePoint[i], 2, cv::Scalar(0, 255, 0), -1);
            }

            cv::line(undistortionImage, cv::Point(undistortionImage.cols / 2, 0), cv::Point(undistortionImage.cols / 2, undistortionImage.rows), cv::Scalar(0, 255, 0), 1);
            cv::line(undistortionImage, cv::Point(0, undistortionImage.rows / 2), cv::Point(undistortionImage.cols, undistortionImage.rows / 2), cv::Scalar(0, 255, 0), 1);

            cv::imshow("perspectiveTransCalibrationWindow", undistortionImage);

            if (mousePoint.size() == 4)
            {
                TransformMat = findHomography(mousePoint, worldPoint, CV_RANSAC);
                shiftTransformMat = findHomography(worldPoint, mousePoint, CV_RANSAC);

                cv::Mat PerspectiveTransformImage;
                cv::warpPerspective(undistortionImage, PerspectiveTransformImage, TransformMat, undistortionImage.size(), CV_INTER_CUBIC);

                cv::imshow("PerspectiveTransformImage", PerspectiveTransformImage);
                cv::waitKey(5000);
                cv::destroyWindow("PerspectiveTransformImage");

                std::ofstream out(PerspectiveTransformCalibration);

                out << TransformMat.at<double>(0, 0) << std::endl;
                out << TransformMat.at<double>(0, 1) << std::endl;
                out << TransformMat.at<double>(0, 2) << std::endl;
                out << TransformMat.at<double>(1, 0) << std::endl;
                out << TransformMat.at<double>(1, 1) << std::endl;
                out << TransformMat.at<double>(1, 2) << std::endl;
                out << TransformMat.at<double>(2, 0) << std::endl;
                out << TransformMat.at<double>(2, 1) << std::endl;
                out << TransformMat.at<double>(2, 2) << std::endl;
                out << shiftTransformMat.at<double>(0, 0) << std::endl;
                out << shiftTransformMat.at<double>(0, 1) << std::endl;
                out << shiftTransformMat.at<double>(0, 2) << std::endl;
                out << shiftTransformMat.at<double>(1, 0) << std::endl;
                out << shiftTransformMat.at<double>(1, 1) << std::endl;
                out << shiftTransformMat.at<double>(1, 2) << std::endl;
                out << shiftTransformMat.at<double>(2, 0) << std::endl;
                out << shiftTransformMat.at<double>(2, 1) << std::endl;
                out << shiftTransformMat.at<double>(2, 2) << std::endl;
                out.close();
                mousePoint.clear();
                CALIBRATION_SUCCESS = true;
            }

        }

    }
    return CALIBRATION_SUCCESS;
#endif
}
