#ifndef K_PERCENT_EXTRACTOR_H_
#define K_PERCENT_EXTRACTOR_H_

#include"LMsfromCam.h"
#include "../3rdParty/LSD/LSDLines.h"
#include "eigen3/Eigen/Dense"
#include <opencv2/core/eigen.hpp>


namespace LD {

    class KPercentExtractor : public LMsfromCam{

    private:

        cv::Mat res;
        Eigen::Vector2f soln;

        //std::vector<std::vector<int> > VPlines;
        double minerr; // the minimum distance error
        int minlength; //store minimum length for lines
        std::vector<int> temp; //temporary vector for intermediate storage

    protected:

        int m_k;

        virtual void ParseXML() override;

    public:

        void runDFS(cv::Mat &_noisefreeImg, int x, int y, int &_nclusters, vector<cv::Point> &_pts);

        int noiseRemover(cv::Mat &_segImg,int maxclusters);

        int VPtsdector(const cv::Mat &_extractedImg, cv::Mat &_refinedImg, cv::Mat &_labeledImg, int thresh);

        ntuple_list callLsd(cv::Mat *src);

        void initVPs(cv::Mat grayImag, std::vector<std::vector<int> > &points);

        void makeLines(Eigen::MatrixXf &A, Eigen::MatrixXf &B, std::vector<std::vector<int> > points);

        int GetImgVP(const cv::Mat _grayimg, cv::Mat &_refinedImg, cv::Mat &labeledImg, Eigen::MatrixXf &A, Eigen::MatrixXf &B, int thresh, std::vector<std::vector<int> > points);

        virtual void Preprocess(const Mat &_original, const Mat &_segImg, Mat &_preprocessed);

        virtual void LMsfromImg(const Mat &_original, const Mat &_segImg, Mat &_refinedImg) override;

        KPercentExtractor(string _xmlFile) : LMsfromCam(_xmlFile) { ParseXML(); }

    };

}

#endif
