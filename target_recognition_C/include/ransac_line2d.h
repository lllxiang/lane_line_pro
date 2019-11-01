#ifndef RANSACLINE2D_H_
#define RANSACLINE2D_H_

#include <stdlib.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <time.h>
#include <iostream>


namespace aps
{
    class LineModel
    {
    public:
        LineModel() :
                mSlope(0), mIntercept(0)
        {
        }

        LineModel(double _slope, double _intercept) :
                mSlope(_slope), mIntercept(_intercept)
        {
        }

        //透视图中的点斜式方程
        double mSlope;
        double mIntercept;
        //IPM中的点斜式方程
        double mSlope_ipm;
        double mIntercept_ipm;

        double alpha;
        double alpha_ipm; //直线与x轴正方向的夹角 0-180
        //两个端点
        std::vector<cv::Point>  p_toushi;      //透视图中点坐标
        std::vector<cv::Point>  p_ipm;    //俯视图中的点坐标
    };

    class RansacLine2D
    {
    public:

        RansacLine2D();

        virtual
        ~RansacLine2D();

        void
        setObservationSet(const std::vector<cv::Point>& observationSet)
        {

            mObservationSet = observationSet;
        }

        void
        setTreshold(const float sigma)
        {
            mThreshold = sigma;
        }

        void
        setIterations(const int Iterations)
        {
            mIterations = Iterations;
        }

        void
        setRequiredInliers(const int requiredInliers)
        {
            mRequiredInliers = requiredInliers;
        }

        bool
        computeModel();

        void
        getBestModel(LineModel& bestLineModel)
        {
            bestLineModel.mSlope = mBestModel.mSlope;
            bestLineModel.mIntercept = mBestModel.mIntercept;
        }

        int
        getBestRank()
        {
            return mBestRank;
        }

    private:
        bool
        isMember(const cv::Point& point,
                 const std::vector<cv::Point>& maybeInliers) const;
        bool
        fitsModel(const cv::Point& point, const LineModel& model) const;

        LineModel
        getModel(const cv::Point& p1, const cv::Point& p2) const;

        LineModel
        getModel(const std::vector<cv::Point>& observation) const;

        std::vector<cv::Point>
        getMaybeInliers() const;

        int
        getModelRank(const std::vector<cv::Point>& pointList,
                     const LineModel& model) const;
        double
        getDistance(const cv::Point& p, const LineModel& model) const;

        bool
        isdegenerate(cv::Point p1, cv::Point p2);


    private:
        std::vector<cv::Point> mObservationSet;      //内点。求内点线段 的两个断点，并作为成员变量。返回。。用于界定引导
        //线的范围。
        std::vector<cv::Point> mBestConsensusSet;

        LineModel mBestModel;
        int mRequiredInliers;
        int mIterations;
        int mBestRank;
        double mThreshold;
    public:
        std::vector<cv::Point> m_notConsensusSet;  //保存 外点。
        cv::Point p_min; //内点 x最小对应的点
        cv::Point p_max; //内点集合 x最大对应的点

    };

}

#endif