#include"LMsintersection.h"
#include"../include/Utilities.h"
#include <gsl/gsl_integration.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_monte.h>
#include <gsl/gsl_monte_plain.h>
#include <gsl/gsl_monte_miser.h>
#include <gsl/gsl_monte_vegas.h>
#include <omp.h>

namespace LD {

    Eigen::MatrixXf LMsintersection::H_sf = Eigen::MatrixXf::Zero(10, 1);
    Eigen::MatrixXf LMsintersection::H_p = Eigen::MatrixXf::Zero(4, 1);

    void LMsintersection::ParseXML() {
        m_xml = m_xml.child("LMintersection");

        m_segRoot = m_xml.attribute("segRoot").as_string();
        m_refinedRoot = m_xml.attribute("refinedRoot").as_string();
        m_segImgPrefix = m_xml.attribute("segImgPrefix").as_string();
        m_refImgPrefix = m_xml.attribute("refImgPrefix").as_string();
        m_outputFile = m_xml.attribute("outputFile").as_string();
        m_saveVizImg = m_xml.attribute("saveVizImg").as_bool();
        m_vizImgPrefix = m_xml.attribute("vizImgPrefix").as_string();
        m_maxWidth = m_xml.attribute("maxWidth").as_int();
        m_maxLength = m_xml.attribute("maxLength").as_int();
        m_printOnly2D = m_xml.attribute("printOnly2D").as_bool(false);
        m_dThrehold = m_xml.attribute("dThrehold").as_float();
        m_nIter = m_xml.attribute("nIter").as_int();

        if (m_segRoot.empty() || m_refinedRoot.empty() || m_segImgPrefix.empty() || m_refImgPrefix.empty() ||
            m_outputFile.empty() || (m_saveVizImg && m_vizImgPrefix.empty()) || !m_maxWidth || !m_maxLength)
            throw runtime_error(
                    "at least one of the following attributes are missing in LMsintersection node: segRoot, refinedRoot, outputFile, segImgPrefix, refImgPrefix, vizImgPrefix, maxWidth, maxHeight");
    }

    void LMsintersection::ProcessProjectedLidarPts(const Eigen::MatrixXf &_veloImg, const Mat &_veloPoints,
                                                   const Mat &_reflectivity, Mat &_inputImg) {

        if (m_debug)
            cout << "Exiting LMintersection::ProcessProjectedLidarPts()" << endl;

        string segImgName = m_segRoot + "/" + m_segImgPrefix + m_imgBaseName;
        string refImgName = m_refinedRoot + "/" + m_refImgPrefix + m_imgBaseName;

        Mat segImg = imread(segImgName, IMREAD_GRAYSCALE);
        Mat refinedImg = imread(refImgName, IMREAD_GRAYSCALE);

        if (segImg.empty())
            throw std::runtime_error("Can't open " + segImgName);
        else if (m_debug)
            cout << "Successfully read segmented image: " << segImgName << endl;

        if (refinedImg.empty())
            throw std::runtime_error("Can't open " + refImgName);
        else if (m_debug)
            cout << "Successfully read refined image: " << refImgName << endl;

        double thresh = OtsuThresholdRoad(_veloImg, segImg, _reflectivity);
        if (m_debug)
            cout << "Threshold set to " << thresh << endl;

        Eigen::ArrayXXf intersectedPts;
        float _areaDifference;

        IntersectIn3D(_veloImg, _veloPoints, _reflectivity, refinedImg, thresh, intersectedPts, _inputImg);
        PrintToFile(intersectedPts);

        ComputeDistanceSum(_veloImg, _veloPoints, segImg, _areaDifference);

        if (m_saveVizImg)
            imwrite(m_outputRoot + "/" + m_vizImgPrefix + m_imgBaseName, _inputImg);

        if (m_debug)
            cout << "Exiting LMintersection::ProcessProjectedLidarPts()" << endl;

    }

    void LMsintersection::operator()(const Mat &_veloPoints, const Mat &_segImg, const Mat &_refinedImg,
                                     Eigen::ArrayXXf &_intersectedPts, Mat &_reflectivity,
                                     Eigen::MatrixXf &_veloImgPts, float &_areaDifference, const Mat &_inputImg) {
        if (m_debug)
            cout << "Entering LMintersection::()" << endl;

        cv::Mat _origImg = _inputImg.clone();

        Project(_veloPoints, _veloImgPts, _reflectivity);

        double thresh = OtsuThresholdRoad(_veloImgPts, _segImg, _reflectivity);

        if (m_debug)
            cout << "Threshold set to " << thresh << endl;

        IntersectIn3D(_veloImgPts, _veloPoints, _reflectivity, _refinedImg, thresh, _intersectedPts, _origImg);

        if (m_debug)
            PrintToFile(_intersectedPts);

        ComputeDistanceSum(_veloImgPts, _veloPoints, _segImg, _areaDifference);

        if (m_debug)
            cout << "Entering LMintersection::()" << endl;
    }

    void
    LMsintersection::IntersectIn3D(const Eigen::MatrixXf _veloImg, const Mat &_veloPoints, const Mat &_reflectivity,
                                   const Mat &_refinedImg, const double &_thresh, Eigen::ArrayXXf &_intersectedPts) {
        Mat test;
        IntersectIn3D(_veloImg, _veloPoints, _reflectivity, _refinedImg, _thresh, _intersectedPts, test);
    }

    void
    LMsintersection::IntersectIn3D(const Eigen::MatrixXf _veloImg, const Mat &_veloPoints, const Mat &_reflectivity,
                                   const Mat &_refinedImg, const double &_thresh, Eigen::ArrayXXf &_intersectedPts,
                                   Mat &_vizImg) {
        if (m_debug)
            cout << "Entering LMintersection::IntersectIn3D()" << endl;

        vector<vector<float> > intersectedPtsVec;

        for (ulli i = 0; i < _veloImg.rows(); i++) {
            int xImg = _veloImg(i, 0), yImg = _veloImg(i, 1);
            float xLidar = _veloPoints.at<float>(i, 0), yLidar = _veloPoints.at<float>(i, 1);
            float reflect = _reflectivity.at<float>(i, 0);
            if (isValid(yImg, xImg, _refinedImg.rows, _refinedImg.cols) && std::abs(yLidar) <= m_maxWidth &&
                std::abs(xLidar) <= m_maxLength &&
                _refinedImg.at<unsigned char>(yImg, xImg) && reflect >= _thresh) {
                if (m_printOnly2D)
                    intersectedPtsVec.push_back({xLidar, yLidar});
                else {
                    intersectedPtsVec.push_back({xLidar, yLidar, _veloPoints.at<float>(i, 2)});
                }
                if (m_debug) {
                    if (m_saveVizImg && !_vizImg.empty())
                        circle(_vizImg, Point(xImg, yImg), 3, Scalar(0, 255, 0), CV_FILLED);
                }
            }
        }

        _intersectedPts.resize(intersectedPtsVec.size() ? intersectedPtsVec[0].size() : 0, intersectedPtsVec.size());

        for (ulli r = 0; r < _intersectedPts.rows(); r++)
            for (ulli c = 0; c < _intersectedPts.cols(); c++)
                _intersectedPts(r, c) = intersectedPtsVec[c][r];

        if (m_debug)
            cout << "Exiting LMintersection::IntersectIn3D()" << endl;
    }

    void LMsintersection::PrintToFile(Eigen::ArrayXXf &_intersectedPts) {
        if (m_debug)
            cout << "Entering LMintersection::PrintToFile()" << endl;

        m_fout << m_imgBaseName << endl;
        m_fout << _intersectedPts.cols() << "\t" << _intersectedPts.rows() << endl;

        for (ulli c = 0; c < _intersectedPts.cols(); c++) {
            for (ulli r = 0; r < _intersectedPts.rows(); r++)
                m_fout << _intersectedPts(r, c) << "\t";

            m_fout << endl;
        }

        if (m_debug)
            cout << "Exiting LMintersection::PrintToFile()" << endl;
    }

    double
    LMsintersection::OtsuThresholdRoad(const Eigen::MatrixXf _veloImg, const Mat &_segImg, const Mat &_reflectivity) {
        if (m_debug)
            cout << "Entering LMintersection::OtsuThresholdRoad()" << endl;

        //Find Otsu thresholding for points that are on road and have positive reflectivity
        Mat scaledReflectivity = 255 * _reflectivity;
        scaledReflectivity.convertTo(scaledReflectivity, CV_8UC1);
        vector<unsigned char> onRoadRef;
        onRoadRef.reserve(scaledReflectivity.rows);

        for (ulli i = 0; i < scaledReflectivity.rows; i++) {
            int x = _veloImg(i, 0), y = _veloImg(i, 1);
            int reflect = scaledReflectivity.at<unsigned char>(i, 0);
            if (isValid(y, x, _segImg.rows, _segImg.cols) && _segImg.at<unsigned char>(y, x))
                onRoadRef.push_back(scaledReflectivity.at<unsigned char>(i, 0));
        }

        double thresh = threshold(onRoadRef, onRoadRef, 1, 255, THRESH_TOZERO | THRESH_OTSU) / 255;

        if (m_debug)
            cout << "Exiting LMintersection::OtsuThresholdRoad()" << endl;

        return thresh;
    }

    double LMsintersection::surfaceModel(double k[], size_t dim, void *params) {
        if (H_sf.rows() != 10) {
            std::cerr << "The dimension is not correct (not 10) in  roadsurfacefint, pleasee check..." << endl;
            exit(1);
        }
        (void) (dim);
        (void) (params);
        return (
                H_sf(0, 0) + H_sf(1, 0) * k[0] + H_sf(2, 0) * k[1] + H_sf(3, 0) * k[0] * k[0] + H_sf(4, 0) * k[0] * k[1]
                + H_sf(5, 0) * k[1] * k[1] + H_sf(6, 0) * pow(k[0], 3) + H_sf(7, 0) * pow(k[0], 2) * k[1] +
                H_sf(8, 0) * pow(k[1], 2) * k[0] + H_sf(9, 0) * pow(k[1], 3));

    }

    // road plane function for integration
    double LMsintersection::planeModel(double k[], size_t dim, void *params) {

        if (H_p.rows() != 4) {
            std::cerr << "H_p: " << H_p
                      << "The coeff's format is not correct (not 4) in  SurfaceDataMaker::roadplanefint, check..."
                      << endl;
            exit(1);
        }
        (void) (dim);
        (void) (params);
        return ((-H_p(3, 0) - H_p(0, 0) * k[0] - H_p(1, 0) * k[1]) / H_p(2, 0));
    }


    double LMsintersection::roadSurfArea(double dx, double dy) {
        if (m_debug)
            cout << "Entering LMintersection::roadSurfArea()" << endl;

        double res, err;
        double xl[2] = {-dx, -dy};
        double xu[2] = {dx, dy};
        size_t calls = 5000000;
        const gsl_rng_type *T;
        gsl_rng *r;
        gsl_monte_function G = {surfaceModel, 2, 0};
        gsl_rng_env_setup();
        T = gsl_rng_default;
        r = gsl_rng_alloc(T);
        gsl_monte_miser_state *s = gsl_monte_miser_alloc(2);
        gsl_monte_miser_integrate(&G, xl, xu, 2, calls, r, s, &res, &err);
        gsl_monte_miser_free(s);
        if (m_debug)
            cout << "Existing LMintersection::roadSurfArea()" << endl;
        return res;
    }


    double LMsintersection::roadPlaneArea(double dx, double dy) {
        if (m_debug)
            cout << "Entering LMintersection::roadPlaneArea()" << endl;
        double res, err;
        double xl[2] = {-dx, -dy};
        double xu[2] = {dx, dy};
        size_t calls = 5000000;
        const gsl_rng_type *T;
        gsl_rng *r;
        gsl_monte_function G = {planeModel, 2, 0};
        gsl_rng_env_setup();
        T = gsl_rng_default;
        r = gsl_rng_alloc(T);
        gsl_monte_miser_state *s = gsl_monte_miser_alloc(2);
        gsl_monte_miser_integrate(&G, xl, xu, 2, calls, r, s, &res, &err);
        gsl_monte_miser_free(s);
        if (m_debug)
            cout << "Existing LMintersection::roadPlaneArea()" << endl;
        return res;
    }

    void LMsintersection::ComputeDistanceSum(const Eigen::MatrixXf &_veloImg, const Mat &_veloPoints,
                                             const Mat &_segImg,
                                             float &distSum) {

        if (m_debug)
            cout << "Entering LMintersection:: ComputeDistanceSum" << endl;

        Mat roadsurfLidarpts;
        for (int i = 0; i < _veloImg.rows(); i++) {
            int x = _veloImg(i, 0), y = _veloImg(i, 1);
            if (isValid(y, x, _segImg.rows, _segImg.cols) && _segImg.at<unsigned char>(y, x) &&
                fabs(_veloPoints.at<unsigned char>(i, 0)) < m_maxLength &&
                fabs(_veloPoints.at<unsigned char>(i, 1)) < m_maxWidth) {
                cv::Mat m = _veloPoints.row(i).colRange(0, 3);
                roadsurfLidarpts.push_back(m);
            }
        }


        Eigen::MatrixXf roadlidarpts;
        cv2eigen(roadsurfLidarpts, roadlidarpts);
        int nSamNumForModel = 10, ninlierThreshold = std::floor(0.95 * roadlidarpts.rows());
        retresult coeffsurface = runRansacRoad(roadlidarpts, nSamNumForModel, m_nIter, ninlierThreshold,
                                               m_dThrehold);
        H_sf = coeffsurface.modelcoff;
        retresult area_surf = mcdcomputeres(coeffsurface.modelcoff, roadlidarpts, m_dThrehold, false);
        float resf = (area_surf.inliers.col(3)).sum();

        // fit a plane to road surface lidar points
        int nsumforplane = 3, ninlierThresholdforPlane = std::floor(0.8 * roadlidarpts.rows());
        m_nIter = min(m_nIter, 200);
        retresult coeffplane = runRansacplanefit(roadlidarpts, nsumforplane, m_nIter, ninlierThresholdforPlane,
                                                 m_dThrehold);
        H_p = coeffplane.modelcoff;
        retresult area_plane = mcdPlaneComput(coeffplane.modelcoff, roadlidarpts, m_dThrehold, false);
        float resp = (area_plane.inliers.col(3)).sum();

        distSum = (float) fabs(fabs(resf) - fabs(resp));

        if (m_debug)
            cout << "The lane surface correctness metric is " << distSum << endl;


        if (m_debug)
            cout << "Exiting LMintersection::ComputeDistanceSum()" << endl;
    }

}
