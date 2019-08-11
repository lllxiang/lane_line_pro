#include <opencv2/opencv.hpp>
//#include <io.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <string.h> //包含strcmp的头文件,也可用: #include <ctring>
#include <dirent.h>


using namespace std;
using namespace cv;

//读取文件夹下的指定类型文件
void getFileNames(const std::string path, std::vector<std::string>& filenames,  std::string suffix)

{
    DIR *pDir;
    struct dirent* ptr;
    if (!(pDir = opendir(path.c_str())))
        return;
    while ((ptr = readdir(pDir))!=0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            std::string file = path + "/" + ptr->d_name;
            if (opendir(file.c_str()))
            {
                getFileNames(file, filenames, suffix);
            }
            else
            {
                if (suffix == file.substr(file.size() - suffix.size()))
                {
                    filenames.push_back(file);
                }
            }
        }
    }
    closedir(pDir);
}



int main()
{
    stringstream file_name;
    string path_name = "/home/baozhengfan/workspace/intrinsic_param_calib_image_input/calibration/fish1"; //保存结果

    file_name << path_name;
    ofstream fout(file_name.str() + "/calibrate_result.txt");
    ofstream fout_extrinsic(file_name.str() + "/extrinsic.txt");
    ofstream fout_instrinsic(file_name.str() + "/intrinsic.txt");
    ofstream fout_distortion(file_name.str() + "/distortion.txt");
    /************************************************************************
           标定布数据
    *************************************************************************/
    std::cout << "开始提取角点­" << std::endl;
    //int image_count=  125;
    Size board_size = Size(9, 6);            //标定布内角点
    Size2f square_size = Size2f(3.7, 3.7);  //20,20
    vector<Point2f> corners;
    vector<vector<Point2f>>  corners_Seq;
    vector<Mat>  image_Seq;
    int successImageNum = 0;
    //int flag_num;

    /******************  read the fixed format file   ********************************/
    string filePath = path_name+"/raw_image";// 图片路径
    vector<string> files;
    string format = ".jpg";
    getFileNames(filePath, files, format);
    int image_count = files.size();
    /********************************************************************************/

    int count = 0;
    for (int i = 0; i != image_count; i++)
    {

        std::cout << "Frame #" << i + 1 << "..." << endl;
        string imageFileName;
        std::stringstream StrStm;
        StrStm << filePath+"/ext_";
        StrStm<<i+1;
        StrStm>>imageFileName;
        imageFileName += ".jpg";
        cv::Mat image = imread(imageFileName);
        //cout<<imageFileName<<endl;
        //cv::Mat image = imread(files[i]);
        if (image.empty())
            cout << "读入失败" << endl;


        Mat imageGray;
        cvtColor(image, imageGray, CV_RGB2GRAY);
        bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
            CALIB_CB_FAST_CHECK);
        if (!patternfound)
        {
            cout << "can not find chessboard corners!\n";
            continue;
            exit(1);
        }
        else
        {
            cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));            
            Mat imageTemp = image.clone();
            for (int j = 0; j < corners.size(); j++)
            {
                circle(imageTemp, corners[j], 5, Scalar(0, 0, 255), 2, 8, 0);
            }
            string imageFileName;
            std::stringstream StrStm;
            StrStm << path_name << "/corners/";
            StrStm << i + 1;
            StrStm >> imageFileName;

            imageFileName += "_corner.jpg";
            imwrite(imageFileName, imageTemp);
            cout << "Frame corner#" << i + 1 << "...end" << endl;

            count = count + corners.size();
            successImageNum = successImageNum + 1;
            corners_Seq.push_back(corners);

        }
        image_Seq.push_back(image);
    }
    cout << "corners detection is over!\n";
    /************************************************************************
         参数标定
    *************************************************************************/
    std::cout << "Calibration Begin­" << std::endl;
    std::vector<std::vector < cv::Point3f >> object_Points;
    Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));
    std::vector<int>  point_counts;
    /* worl coordinate system */
    for (int t = 0; t < successImageNum; t++)
    {
        vector<Point3f> tempPointSet;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {                
                Point3f tempPoint;
                tempPoint.x = (float)i*square_size.width;
                tempPoint.y = (float)j*square_size.height;
                tempPoint.z = 0;
                tempPointSet.push_back(tempPoint);
            }
        }
        object_Points.push_back(tempPointSet);
    }
    for (int i = 0; i < successImageNum; i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    /* 开始标定 */
    Size image_size = image_Seq[0].size();
    cv::Mat intrinsic_matrix(cv::Size(3, 3), CV_64FC1); //cv::Matx33d intrinsic_matrix;
    cv::Vec4d distortion_coeffs;
    std::vector<cv::Vec3d> rotation_vectors;
    std::vector<cv::Vec3d> translation_vectors;
    int flags = 0;
    flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= cv::fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
    fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
    cout << "Calibration is over!\n";

    /************************************************************************
           精度计算
    *************************************************************************/
    cout << "Calculate the error!­" << endl;
    double total_err = 0.0;
    double err = 0.0;
    vector<Point2f>  image_points2;

    for (int i = 0; i < successImageNum; i++)
    {
        vector<Point3f> tempPointSet = object_Points[i];        
        fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs);        
        vector<Point2f> tempImagePoint = corners_Seq[i];
        Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
        Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
        for (size_t i = 0; i != tempImagePoint.size(); i++)
        {
            image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
            tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err /= point_counts[i];
        cout << "第" << i + 1 << "幅图像的平均误差" << err << "像素" << endl;
        fout << "第" << i + 1 << "幅图像的平均误差" << err << "像素" << endl;
    }
    cout << "总体平均误差" << total_err / image_count << "像素" << endl;
    fout << "总体平均误差" << total_err / image_count << "像素" << endl << endl;
    cout << "误差评价完成!" << endl;

    /************************************************************************
           保存标定结果
    *************************************************************************/
    cout << "开始保存标定结果­" << endl;
    Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));

    fout << "相机内参数矩阵:" << endl;
    fout << intrinsic_matrix << endl;
    fout_instrinsic << intrinsic_matrix.at<double>(0, 0) << std::endl;
    fout_instrinsic << intrinsic_matrix.at<double>(0, 2) << std::endl;
    fout_instrinsic << intrinsic_matrix.at<double>(1, 1) << std::endl;
    fout_instrinsic << intrinsic_matrix.at<double>(1, 2) << std::endl;

    fout << "畸变系数:";
    fout << distortion_coeffs << std::endl;
    fout_distortion << distortion_coeffs[0] << std::endl;
    fout_distortion << distortion_coeffs[1] << std::endl;
    fout_distortion << distortion_coeffs[2] << std::endl;
    fout_distortion << distortion_coeffs[3] << std::endl;

    for (int i = 0; i < successImageNum; i++)
    {
        fout << "第" << i + 1 << "幅图像的旋转向量" << endl;
        fout << rotation_vectors[i] << endl;

        /* 罗德李格分解*/
        Rodrigues(rotation_vectors[i], rotation_matrix);
        fout << "第" << i + 1 << "幅图像的旋转矩阵" << endl;
        fout << rotation_matrix << endl;
        fout << "第" << i + 1 << "幅图像的平移向量" << endl;
        fout << translation_vectors[i] << endl;
        if (i == successImageNum-1)// save the last extrinsic
        {
            fout_extrinsic << rotation_vectors[i][0] << std::endl;
            fout_extrinsic << rotation_vectors[i][1] << std::endl;
            fout_extrinsic << rotation_vectors[i][2] << std::endl;
            fout_extrinsic << translation_vectors[i][0] << std::endl;
            fout_extrinsic << translation_vectors[i][1] << std::endl;
            fout_extrinsic << translation_vectors[i][2] << std::endl;
        }
    }
    cout << "完成保存" << endl;
    fout.close();
    fout_extrinsic.close();


    /************************************************************************
          显示标定结果
    *************************************************************************/
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);

    cout << "保存矫正图像" << endl;
    for (int i = 0; i != successImageNum; i++)
    {
        cout << "Frame #" << i + 1 << "..." << endl;
        Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
        fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
            getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0), image_size, CV_32FC1, mapx, mapy);
        Mat t = image_Seq[i].clone();
        cv::remap(image_Seq[i], t, mapx, mapy, INTER_LINEAR);
        string imageFileName;
        std::stringstream StrStm;
        StrStm << path_name << "/undistorted/";
        StrStm << i + 1;

        StrStm >> imageFileName;

        imageFileName += "_d.jpg";
        imwrite(imageFileName, t);
    }
    cout << "保存结束" << endl;
return 0;
}

