#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "c_fun.hpp"
using namespace std;
using namespace cv;
//#define unsigned int uint8_t;

int main() {
    std::string img_dir = "/home/lx/mnt/docker_data_4010/docker_data/surround_ipm7_test/mlednet/20191022060433/8left/pred_image_png/im_0.png";
    cv::Mat im_src = cv::imread(img_dir, cv::IMREAD_GRAYSCALE);
    unsigned int img_width = 320;
    unsigned int img_height = 256;
    for(int i=0; i<img_height; i++)
    {
        for (int j = 0; j < img_width; j++)
        {
            if (im_src.at<uchar >(i, j) == 4)
            {
                im_src.at<uchar >(i, j) = 255;
            }
        }
        //cout<<endl;
    }
    uint32_t  a;

    uint8_t img_ps[img_width*img_height] = {0,};
    //img_uint8赋值
    for(int i=0; i<img_height; i++)
    {
        for (int j = 0; j < img_width; j++)
        {
            if (im_src.at<uchar >(i, j) == 255)
            {
                img_ps[j + i * img_width] = 1;
            }
            cout<<(int)img_ps[j + i * img_width]<<",";
        }
    }
    cout<<"-------------------------------------------------------------\n";
    //四邻域查找边缘
    //寄存器变量，临时保存副本
    double t = (double)cvGetTickCount();
    uint32_t p_img_w[1] = {320};
    uint32_t p_img_h[1] = {256};

    //查找边缘
    PSP_find_edges_in_bimg(img_ps,
                           p_img_w,
                           p_img_h,
                           1);
    //Hough 并将结果保存至 []
    cv::Mat im_hough = im_src.clone();
    cv::Mat im_show = im_src.clone();
    for(int i=0; i<img_height; i++)
    {
        for (int j = 0; j < img_width; j++)
        {
            im_hough.at<uchar>(i, j) = img_ps[j + i * img_width] * 255;
            im_show.at<uchar>(i, j) = img_ps[j + i * img_width] * 255;
        }
    }




    test_fun();

    t = ((double)cvGetTickCount() - t)/cvGetTickFrequency();
    printf("duration = %fus\n", t);
    //显示结果
    cv::imshow("im_show", im_show);
    cv::imshow("im_src", im_src);
    cv::waitKey(0);
}
