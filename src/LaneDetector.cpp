#include"../include/LaneDetector.h"
#include "../3rdParty/TLinkage/Sampler.h"
#include<fstream>
#include<libgen.h>
#include <chrono>
#include <ratio>
#include <thread>

namespace LD {

    LaneDetector::LaneDetector(string _xmlFile) : Solver(_xmlFile), m_RoadSegment(_xmlFile), m_LMsfromCam(_xmlFile),
                                                  m_LMsintersection(_xmlFile), m_bSplineTLinkage(_xmlFile),
                                                  m_laneQualityChecker(_xmlFile), m_visualizer(_xmlFile) {
        ParseXML();
        assert(!m_LMsintersection.isMode2D());
    }

    void LaneDetector::ParseXML(){
        if (m_debug)
            cout << "Entering LaneDetector::ParseXML()" << endl;

        m_xml = m_xml.child("LaneDetector");

        m_dataRoot = m_xml.attribute("dataRoot").as_string();
        m_dataFile = m_xml.attribute("dataFile").as_string();
        m_veloRoot = m_xml.attribute("veloRoot").as_string();
        m_ratiosFile = m_xml.attribute("ratiosFile").as_string();
        m_saveVizImg = m_xml.attribute("saveVizImg").as_bool(true);
        m_vizImgPrefix = m_xml.attribute("vizImgPrefix").as_string();
        m_laneAssess = m_xml.attribute("lassAsses").as_bool();

        if (m_dataRoot.empty() || m_dataRoot.empty() || m_veloRoot.empty() || m_ratiosFile.empty() ||
            (m_saveVizImg && m_vizImgPrefix.empty()))
            throw runtime_error(
                    "at least one of the following attributes is missing: imgRoot, imgFile, veloRoot, ratiosFile, vizImgPrefix, saveVizImg");

        if (m_debug)
            cout << "Existing LaneDetector::ParseXML()" << endl;
    }

    void LaneDetector::operator()(const cv::Mat &_inputImg, const cv::Mat &_veloPoints, vector<Eigen::ArrayXf> &_models,
                                  float &_brightnessRatio, float &_reflectivityRatio, float &_areaDifference,
                                  float &_AverageDistance) {
        if (m_debug)
            cout << "Entering LaneDetector::()" << endl;

        cv::Mat segImg, refinedImg;
        Eigen::ArrayXXf intersectedPts;
        Eigen::ArrayXf clusters;
        Mat reflectivity;
        Eigen::MatrixXf veloImg;
        m_RoadSegment(_inputImg, segImg);
        m_LMsfromCam(_inputImg, segImg, refinedImg);
        m_LMsintersection(_veloPoints, segImg, refinedImg, intersectedPts, reflectivity, veloImg, _areaDifference,
                          _inputImg);
        m_bSplineTLinkage(intersectedPts, clusters, _models);

        if (m_laneAssess) {
            m_laneQualityChecker(intersectedPts, clusters, _veloPoints, reflectivity, veloImg, _inputImg, segImg,
                                 refinedImg, _brightnessRatio, _reflectivityRatio);
            int npt = 1;
            double sum = 0;
            for (int i = 0; i < _models.size(); i++) {
                for (int j = 0; j < intersectedPts.cols(); ++j) {
                    double PtsToSpline = m_bSplineTLinkage.Distance(
                            intersectedPts.block(0, j, intersectedPts.rows(), 1),
                            _models[i]);
                    if (PtsToSpline <= 0.3) {
                        sum += PtsToSpline;
                        npt++;
                    }
                }

            }
            _AverageDistance = (float) sum / npt; // the shape metric
        }

        if (m_saveVizImg) {
            for (int i = 0; i < _models.size(); i++) {
                ArrayXXf coordinates;
                m_bSplineTLinkage.VisualizeModel(_models[i], coordinates);
                Mat vizImg = _inputImg;
                m_visualizer(vizImg, coordinates);
                imwrite(m_vizImgPrefix + m_imgBaseName, _inputImg);
            }
        }

        if (m_debug)
            cout << "Exiting LaneDetector::()" << endl;
    }

    float LaneDetector::computeLazyParas(std::deque<float> Area) {
        if (m_debug)
            cout << "Entering LaneDetector::computeLazyParas" << endl;

        auto minx = std::min_element(Area.begin(), Area.end());
        float area_1 = Area.back();
        Area.pop_back();
        float area_2 = Area.back();
        float res;
        if (std::fabs(area_1 - *minx) != 0) res = (std::fabs(area_1 - area_2)) / (std::fabs(area_1 - *minx));
        else res = 0;

        if (m_debug)
            cout << "Existing LaneDetector::computeLazyParas" << endl;

        return res;
    }

    void LaneDetector::Run() {
        if (m_debug)
            cout << "Entering LaneDetector::Run()" << endl;

        std::ifstream fin(m_dataFile.c_str());
        std::ofstream fout(m_ratiosFile.c_str());
        string line;
        ulli index = 0;
        std::deque<float> lazyMat;
        int lazysize = 3;
        while (std::getline(fin, line)) {
            float brightnessRatio = 0, reflectivityRatio = 0, areaDifference = 0, AverageDistance = 0;
            m_imgBaseName = basename(const_cast<char *>(line.c_str()));
            cv::Mat inputImg, veloPoints;
            vector<Eigen::ArrayXf> models;
            inputImg = cv::imread(m_dataRoot + "/" + line);
            m_LMsintersection.ReadVeloData(m_veloRoot + "/" + line.substr(0, line.size() - 3) + "bin", veloPoints);
            try {
                auto start = std::chrono::steady_clock::now();
                this->operator()(inputImg, veloPoints, models, brightnessRatio, reflectivityRatio, areaDifference,
                                 AverageDistance);
                auto finish = std::chrono::steady_clock::now();

                if (m_debug)
                    cout << "It takes " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count()
                         << " milliseconds to process the Image " << m_imgBaseName << endl;

                m_bSplineTLinkage.PrintModelsToFile(models, m_imgBaseName);

                fout << m_imgBaseName << endl;

                if (m_laneAssess) {
                    if (lazyMat.size() < lazysize) {
                        lazyMat.push_back(areaDifference);
                        lazyparameter = 1;
                    } else {
                        lazyMat.pop_front();
                        lazyMat.push_back(areaDifference);
                        lazyparameter = computeLazyParas(lazyMat);
                    }

                    if (index == 0)
                        AreaDeriva = areaDifference;
                    index += 1;


                    fout << max(brightnessRatio, reflectivityRatio) << endl;
                    fout << AverageDistance << endl;
                    fout << (float) areaDifference / AreaDeriva << endl;
                }
            }
            catch (Sampler::MinSamplesNotFound) {
                fout << m_imgBaseName << endl;
                fout << "---------------------WARNING: Could not find enough samples-----------" << endl;
            }
            catch (alglib::ap_error &e) {
                fout << m_imgBaseName << endl;
                fout << "---------------------WARNING: ALGLIB Error: " << e.msg << "-----------" << endl;
            }
            catch (...) {
                fout << m_imgBaseName << endl;
                fout << "----------------------------Warning: Unknown Error------------------------------" << endl;
            }
        }

        if (m_debug)
            cout << "Exiting LaneDetector::Run()" << endl;
    }
}
