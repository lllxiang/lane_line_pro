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
const double view_dis = 600.0; //可视距离 800cm

//输入：方向标识，type,原图，
        //输出，IPM图
int ipm_trans(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp);
//威视相机外参为止情况下，通过手工选点，估计IPM m矩阵
int ipm_trans_weishi(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp);
bool get_mask_img(cv::Mat & src,cv::Mat &dst, int id);
//原图坐标->去畸变图坐标
bool src2dist(std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
    cv::Mat & mapx, cv::Mat & mapy);
bool read_cam_params(std::string &d);

bool detect_line(cv::Mat & line_pred, std::vector<line_info> & lines);

bool fit_one_line(std::vector<cv::Point> &c, line_info & lines);

bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A);
int solve_super_rect(super_rect & sr);



int main()
{

    struct dirent *dirp;
    std::string img_labeled_dir = "/home/lx/data/surround_line14_test/lednet/6left/6left-labeled/";
    std::string img_pred_dir = "/home/lx/data/surround_line14_test/lednet/6left/6left-pred/";
    std::string img_labeled_all_dir, img_pred_all_dir;
    DIR* dir = opendir(img_labeled_dir.c_str());
    cv::Mat src_img,src_pred;
    int nImg = 14;
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
            std::string direction = "ft";
            nImg++;
                    //读取相机参数
            read_cam_params(direction);
            //读取图片
            src_img = cv::imread(img_labeled_all_dir);
            src_pred = cv::imread(img_pred_all_dir,cv::IMREAD_GRAYSCALE);
            //提取目标类别
            cv::Mat line_pred = cv::Mat::zeros(src_pred.size(),src_pred.type());
            //IPM
            cv::Mat src_img_ipm,line_pred_ipm;
            get_mask_img(src_pred,line_pred,6); //引导线
            int type = 1;
            ipm_trans(direction,type, line_pred, line_pred_ipm);
            ipm_trans(direction,type, src_img, src_img_ipm);


            //车位线检测--车位搜索
            parking_space  ps_search; //实例化车位线对象
            ps_search.img_ps_mask = line_pred; //引导线
            ps_search.img_ps_mask_ipm = line_pred_ipm;
            ps_search.img_ps_bgr = src_img;

            ps_search.detect(); //执行检测操作。

            ps_search.show();

//
//
//
//            double mins = 300; //最小像素轮廓
//            parking_space_info tmp;
//            std::vector<parking_space_info> parking_space; //一帧图像中所有的停车位
//            //find contours
//            std::vector<std::vector<cv::Point>> contours; //contours[0] is the first contours
//            std::vector<std::vector<cv::Point>> contours_filtered;
//            std::vector<cv::Vec4i> hierarchy;
//            cv::findContours(line_pred, contours, hierarchy, cv::RETR_EXTERNAL,
//                             cv::CHAIN_APPROX_NONE );
//            printf("滤波前所有的轮廓 n=%d\n",(int)contours.size());
//            for(int i=0;i<contours.size();i++) //对所有的轮廓
//            {
//                int s = cv::contourArea(contours[i]);
//                int len = cv::arcLength(contours[i],1);
//                if( s < mins)
//                {
//
//                } else{
//                    contours_filtered.push_back(contours[i]);
//                    printf("s=%d\n",s);
//                    printf("len=%d\n",len);
//                }
//            }
//
//
//            //椭圆
//            // 求轮廓的最小外接矩形,并求长短轴、主方向
//            // std::vector<cv::RotatedRect> box(contours_filtered.size());
//            std::vector<super_rect> box;
//            printf("box.size()=%d\n", (int)box.size());
//            cv::Mat paramsA;
//            std::vector<cv::Point> points_fitted;
//            for(int i=0;i<contours_filtered.size();i++)
//            {
//                super_rect tmp;
//                tmp.baseRect = cv::minAreaRect(cv::Mat(contours_filtered[i]));
//                solve_super_rect(tmp); //baseRect -> 长轴,短轴,长轴方向.
//                box.push_back(tmp);
//                printf("box.size()=%d\n", (int)box.size());
//
////                //外接椭圆
////                cv::RotatedRect bcircle; cv::Point2f rect[4];
////                bcircle = cv::fitEllipse(contours_filtered[i]);
////                bcircle.points(rect);
////
////                cv::Point p1((rect[0].x + rect[3].x)/2,(rect[0].y + rect[3].y)/2);
////                cv::Point p2((rect[1].x + rect[2].x)/2,(rect[1].y + rect[2].y)/2);
////                cv::line(src_img_ipm, p1, p2, cv::Scalar(255, 0, 0), 1, 8);
////
////                cv::Point p3((rect[0].x + rect[1].x)/2,(rect[0].y + rect[1].y)/2);
////                cv::Point p4((rect[2].x + rect[3].x)/2,(rect[2].y + rect[3].y)/2);
////                cv::line(src_img_ipm, p3, p4, cv::Scalar(255, 0, 0), 1, 8);
////
////
////                for(int j=0; j<4; j++)
////                {
////                    cv::line(src_img_ipm, rect[j], rect[(j+1)%4], cv::Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
////                }
////                //最小外接矩形
////                bcircle = cv::minAreaRect(cv::Mat(contours_filtered[i]));
////                bcircle.points(rect);
////                for(int j=0; j<4; j++)
////                {
////                    cv::line(src_img_ipm, rect[j], rect[(j+1)%4], cv::Scalar(255, 0, 0), 1, 8);  //绘制最小外接矩形每条边
////                }
//
//                //ransac 拟合直线
////                aps::RansacLine2D ransac_lines;
////                ransac_lines.setObservationSet(tu_contours);
////                ransac_lines.setRequiredInliers(int(3));
////                ransac_lines.setIterations(20);
////                ransac_lines.setTreshold(3);
////                double start = static_cast<double>(cvGetTickCount());
////                ransac_lines.computeModel();
////                double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
////                std::cout << "所用时间为:" << time/1000<<"ms"<<std::endl;
////
////                aps::LineModel fit_line;
////                ransac_lines.getBestModel(fit_line);
////                cv::line(src_img,cv::Point(0,0*fit_line.mSlope+fit_line.mIntercept),
////                                 cv::Point(640,640*fit_line.mSlope+fit_line.mIntercept),cv::Scalar(0,0,255),2);
////
////
////
////                //polynomial_curve_fit(contours_filtered[i], 1, paramsA);
////                cv::Vec4f fitline;
////                //拟合方法采用最小二乘法
////                // cv::fitLine(contours_filtered[i],fitline,CV_DIST_HUBER,0,0.001,0.001);
////                double k_line = fitline[1]/fitline[0];
////                //cv::Point p1(0,k_line*(0 - fitline[2]) + fitline[3]);
////                //cv::Point p2(640 - 1,k_line*(640 - 1 - fitline[2]) + fitline[3]);
////                //cv::line(src_img,p1,p2,cv::Scalar(0,0,255),2);
////
////
//////                for (int x = 0; x < 640; x=x+5)
//////                {
//////                    double y = paramsA.at<double>(0,0) + paramsA.at<double>(0,1) * x +
//////                               paramsA.at<double>(0,2)*std::pow(x, 2);
//////                    points_fitted.push_back(cv::Point(x, y));
//////                }
////                //cv::polylines(src_img, points_fitted, false, cv::Scalar(255, 0, 0), 1, 8, 0); //蓝色 类型二
//
//
//            }
//
//
//
//            //根据距离约束、平行度约束两个特征进行聚类。 每一个车位信息结构体
//            // 包含两条线，代表一个车位存在的位置，
//            // 孤立的引导线被删除
//            int i=0; //考察第0个引导线
//            double disptol=0, para=0;
//            double start_dis = 0;
//            printf("box.size()=%d\n",(int)box.size());
//            if (box.size() > 1)
//            {
//                for(int j=0; j<box.size();j++)
//                {
//                    printf("j: l_edge_point: %d ,x= %d, y= %d \n", j, box[j].l_edge_point[0].x, box[j].l_edge_point[0].y);
//                    for (int i = j+1; i < box.size(); i++)
//                    {
//                        //printf("i: l_edge_point: %d ,x= %d, y= %d \n", i, box[i].l_edge_point[0].x, box[i].l_edge_point[0].y);
//                        //printf("index=%d, x=%d \n", i, box[i].l_edge_point[0].x);
//                        disptol = getDist_P2L(cv::Point(box[j].baseRect.center), box[i].l_edge_point[0],
//                                              box[i].l_edge_point[1]);
//                        para = abs(box[j].l_edge_angle - box[i].l_edge_angle);
//
//                        //start_dis = calu_dis(box[j].l_edge_point[0], box[i].l_edge_point[0]);
//
//                        printf("i = %d disptol=%f,para=%f, start_dis=%f \n",i,  disptol, para, start_dis);
//                        if ((disptol > 110 && disptol < 180) && (para < 30) && (start_dis < 200) )
//                        {
//                            //将两条线信息->车位信息。
//                            //cv::line(src_img_ipm, )
//                            cv::Scalar c = cv::Scalar(rand()%255, rand()%255, rand()%255);
//                            //cv::line(src_img_ipm, box[i].l_edge_point[0], box[i].l_edge_point[1],c, 2, 8, 0);
//                            //cv::line(src_img_ipm, box[j].l_edge_point[0], box[j].l_edge_point[1],c, 2, 8, 0);
//                            tmp.sr1 = box[j];
//                            tmp.sr2 = box[i];
//                            parking_space.push_back(tmp);
//                            //cv::imshow("src_img_ipm", src_img_ipm);
//                            cv::waitKey(0);
//                        }
//                    }
//                }
//            }
//            //聚类完成
//            for (int i=0; i<parking_space.size();i++)
//            {
//                cv::line(src_img_ipm,parking_space[i].sr1.l_edge_point[0],parking_space[i].sr1.l_edge_point[1],cv::Scalar(255,255,0),1,8,0);
//                cv::line(src_img_ipm,parking_space[i].sr1.l_edge_point[1],parking_space[i].sr2.l_edge_point[1],cv::Scalar(255,255,0),1,8,0);
//                cv::line(src_img_ipm,parking_space[i].sr2.l_edge_point[1],parking_space[i].sr2.l_edge_point[0],cv::Scalar(255,255,0),1,8,0);
//                cv::line(src_img_ipm,parking_space[i].sr2.l_edge_point[0],parking_space[i].sr1.l_edge_point[0],cv::Scalar(255,255,0),1,8,0);
//            }
//
//
//            //查看所有的外接矩形
//            printf("滤波后所有的轮廓 n=%d\n",(int)contours_filtered.size());
//            for(int i=0;i<box.size();i++) //对所有的轮廓
//            {
//                cv::Point2f rect[4];
//                box[i].baseRect.points(rect);  //把最小外接矩形四个端点复制给rect数组
//                for(int j=0; j<4; j++)
//                {
//                    //cv::line(src_img_ipm, rect[j], rect[(j+1)%4], cv::Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
//                }
//                //cv::line(src_img, box[i].l_edge_point[0], box[i].l_edge_point[1], cv::Scalar(255, 0, 0), 3, 8);
//                printf("外接举行的角度n=%f,w=%f,h=%f\n", box[i].l_edge_angle,
//                       box[i].s_edge,box[i].l_edge);
//            }
//
//
//            double dis = getDist_P2L(cv::Point(10,10), cv::Point(0,40), cv::Point(40,0));
//            printf("dis = %f\n", dis);
//
////            //在原始图中检测直线
////            std::vector<line_info> lines;
////
////            detect_line(line_pred, lines); //使用轮廓查找方式检测直线
////
////            if(lines.size() == 1)
////            {
////                cv::Mat params;
////                std::vector<cv::Point> points_fitted;
////                std::cout<<"lines[0].line_type"<<lines[0].line_type<<std::endl;
////                if (lines[0].line_type == 1)
////                {
////                    params = lines[0].A1;
////                    for (int x = lines[0].minp; x < lines[0].maxp; x++)
////                    {
////                        double y = params.at<double>(0,0) + params.at<double>(0,1) * x +
////                                   params.at<double>(0,2)*std::pow(x, 2) + params.at<double>(0,3)*std::pow(x, 3)+
////                                params.at<double>(0,4)*std::pow(x, 4) + params.at<double>(0,5)*std::pow(x, 5);
////                        points_fitted.push_back(cv::Point(x, y));
////                    }
////                    cv::polylines(src_img, points_fitted, false, cv::Scalar(0, 0, 255), 1, 8, 0); //红色 类型一
////                }
////                else
////                {
////                    params = lines[0].A2;
////                    for (int y = lines[0].minp; y < lines[0].maxp; y++)
////                    {
////                        double x = params.at<double>(0,0) + params.at<double>(0,1) * y +
////                                   params.at<double>(0,2)*std::pow(y, 2) + params.at<double>(0,3)*std::pow(y, 3)
////                                +params.at<double>(0,4)*std::pow(y, 4)+params.at<double>(0,5)*std::pow(y, 5);
////                        points_fitted.push_back(cv::Point(x, y));
////                    }
////                    cv::polylines(src_img, points_fitted, false, cv::Scalar(255, 0, 0), 1, 8, 0); //蓝色 类型二
////                }
////            }
//
//            cv::imshow("src_img", src_img);
//            //cv::imshow("src_pred", src_pred);
//            cv::imshow("line_pred", line_pred);
//            cv::imshow("src_img_ipm", src_img_ipm);
//            //cv::imshow("line_pred_ipm", line_pred_ipm);
//
//
//            cv::waitKey(0);
//            //std::cout<<"img_labeled_all_dir"<<img_labeled_all_dir << std::endl;
//            //std::cout<<"img_pred_all_dir"<<img_pred_all_dir << std::endl;
//            //std::cout<<"numof_"<<numof_ << std::endl;
//           // std::cout<<"d: "<<tname.substr(numof_ - 2 ,2) << std::endl;

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
int ipm_trans_weishi(std::string driection,int & type, cv::Mat & img_jpg, cv::Mat &warp)
{

    std::vector<cv::Point2f> dst_dist; //俯视图 2D
    std::vector<cv::Point2f> wordPoints;

    //wordPoints.push_back(cv::Point2f());

    cv::Mat m = cv::getPerspectiveTransform(dst_dist,wordPoints);
    //cv::warpPerspective(img_jpg, warp, m, ipm_size);

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
            wordPoints2D[i].x = (wordPoints2D[i].x + (view_dis + (car_len-car_zhouju)/2 + 106))/2;
            wordPoints2D[i].y = (wordPoints2D[i].y + (view_dis - 103))/2;
            ipm_size = cv::Size((view_dis*2 + car_len)/2,  (view_dis+car_weight/2)/2);
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
        cv::warpPerspective(img_jpg, warp, m, ipm_size);
    }
    else
    {
        cv::Mat m = cv::getPerspectiveTransform(distImgPoint, wordPoints2D);
        cv::warpPerspective(img_dist, warp, m, ipm_size);
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

bool detect_line(cv::Mat & line_pred, std::vector<line_info> & lines)
{
    ///std::cout<<"detect_line"<<std::endl;
    double min_s = 30; //面积阈值
    //find contours
    std::vector<std::vector<cv::Point>> contours; //contours[0] is the first contours
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(line_pred, contours, hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_NONE);

    if(contours.size() == 0)
    {
        //std::cout<<"No any line in image."<<std::endl;
    }else if(contours.size() == 1) //只有一个轮廓，滤波后直接拟合
    {
        line_info one_line;
        int c_s = cv::contourArea(contours[0]); //find contours
        if (c_s > min_s)
        {
            //拟合.
            //std::cout<<"is fitting..."<<std::endl;
            //contours[0] ->  params.
            cv::Mat A;
            fit_one_line(contours[0], one_line);

            //std::cout<< "A" << one_line.A1 << std::endl;
            lines.push_back(one_line);
        }
        else
        {
            contours.pop_back();
            //std::cout<<"No any line in image."<<std::endl;
        }
        //int cl = cv::arcLength(contours[0]); //calu contours length
    } else  //轮廓大于一个，滤波->聚类后再拟合
    {

    }
    return 1;
}
bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
    //Number of key points
    int N = key_point.size();
    //构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                                     std::pow(key_point[k].x, i + j);
            }
        }
    }

    //构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                                 std::pow(key_point[k].x, i) * key_point[k].y;
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //求解矩阵A
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}

bool fit_one_line(std::vector<cv::Point> &c, line_info & lines)
{
    //输入 一组无序点集，分别拟合"X"型及"Y"型五次多项式曲线。
    //并选取误差最小的一组参数，作为单白线的曲线参数返回。
    std::vector<cv::Point> c2;
    for(int i=0; i<c.size();i++)
    {
        c2.push_back(cv::Point(c[i].y, c[i].x));
    }
    cv::Mat A1, A2;
    polynomial_curve_fit(c,5, A1); //X型拟合
    polynomial_curve_fit(c2,5, A2); //Y型拟合
    double s1=0, s2=0;
    int xmin=2000, xmax=0;
    int ymin=2000, ymax=0;
    for(int i = 0; i<c.size();i+=2)
    {
        int x =c[i].x;
        double y1 = A1.at<double>(0,0) + A1.at<double>(0,1) * x +
                A1.at<double>(0,2)*std::pow(x, 2) + A1.at<double>(0,3)*std::pow(x, 3) +
                                                    A1.at<double>(0,4)*std::pow(x, 4) +
                                                    A1.at<double>(0,5)*std::pow(x, 5);
        s1 = s1 + abs(y1 - c[i].y);
        if(x<xmin)
        { xmin=x; }
        if(x>xmax)
        { xmax=x; }
    }
    for(int i = 0; i<c2.size();i+=2)
    {
        int y =c[i].y;
        double x = A2.at<double>(0,0) + A2.at<double>(0,1) * y +
                    A2.at<double>(0,2)*std::pow(y, 2) + A2.at<double>(0,3)*std::pow(y, 3)
                        +A2.at<double>(0,4)*std::pow(y, 4) + A2.at<double>(0,5)*std::pow(y, 5);
        s2 = s2 + abs(x - c[i].x);
        if(y<ymin)
        { ymin=y; }
        if(y>ymax)
        { ymax=y; }
    }
    //std::cout<<"s1:"<<s1<<std::endl;
    //std::cout<<"s2:"<<s2<<std::endl;
    lines.A1 = A1;
    lines.A2 = A2;
    if(s1 < s2)
    {
        //X,同时查找端点并返回
        lines.line_type = 1;
        lines.minp = xmin;
        lines.maxp = xmax;
    } else
    {
        lines.line_type = 0;
        lines.minp = ymin;
        lines.maxp = ymax;
    }
    return 1;
}

int solve_super_rect(super_rect & sr)
{
    cv::Point2f rect[4];
    sr.baseRect.points(rect);  //把最小外接矩形四个端点复制给rect数组

    if(sr.baseRect.size.width < sr.baseRect.size.height)
    {
        sr.l_edge = sr.baseRect.size.height;
        sr.s_edge = sr.baseRect.size.width;
        sr.l_edge_angle = 90 - sr.baseRect.angle;
        sr.l_edge_point.push_back(rect[0]);   sr.l_edge_point.push_back(rect[1]); //左下，左上
        sr.l_edge_point.push_back(rect[2]);   sr.l_edge_point.push_back(rect[3]);
    }
    else
    {
        sr.s_edge = sr.baseRect.size.height;
        sr.l_edge = sr.baseRect.size.width;
        sr.l_edge_angle = -sr.baseRect.angle;
        sr.l_edge_point.push_back(rect[1]);   sr.l_edge_point.push_back(rect[2]);
        sr.l_edge_point.push_back(rect[3]);   sr.l_edge_point.push_back(rect[0]);
    }
    return 1;
}


