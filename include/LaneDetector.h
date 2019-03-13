#ifndef LANE_DETECTOR_H_
#define LANE_DETECTOR_H_

#include"Solver.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/ml/ml.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<eigen3/Eigen/Dense>
#include <chrono>
#include"RoadSegment.h"
#include"KPercentExtractor.h"
#include"LMsintersection.h"
#include"../3rdParty/TLinkage/BSplineTLinkage.h"
#include"LaneQualityChecker.h"
#include"PointsVisualizer.h"

namespace LD {
	class LaneDetector : public Solver {
	public:
		void operator()(const cv::Mat &_inputImg, const cv::Mat &_veloPoints, vector<Eigen::ArrayXf> &_models,
						float &_brightnessRatio, float &_reflectivityRatio, float &_areaDifference,
						float &_AverageDistance);

		virtual void Run() override;

		LaneDetector(string _xmlFile);

	protected:
        RoadSegment m_RoadSegment;
		KPercentExtractor m_LMsfromCam;
		LMsintersection m_LMsintersection;
		BSplineTLinkage m_bSplineTLinkage;
		LaneQualityChecker m_laneQualityChecker;
		PointsVisualizer m_visualizer;

		string m_dataFile, m_dataRoot, m_veloRoot;
		string m_imgBaseName;
		string m_ratiosFile;
		bool m_saveVizImg;
		string m_vizImgPrefix;
		float AreaDeriva;
		float computeLazyParas(std::deque<float> Area);
		virtual void ParseXML() override;
	};
}

#endif
