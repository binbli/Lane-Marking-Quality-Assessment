#ifndef LMs_INTERSECTION_H_
#define LMs_INTERSECTION_H_

#include"VeloPtsProjectCam.h"
#include"Utilities.h"

//TODO: Remove repetition of code when counting number of rows in IntersectIn3D

namespace LD {
	class LMsintersection : public VeloPtsProjectCam {
	public:

		static Eigen::MatrixXf H_sf;
		static Eigen::MatrixXf H_p;

		LMsintersection(string _xmlFile) : VeloPtsProjectCam(_xmlFile) {
			ParseXML();
			m_fout.open(m_outputRoot + "/" + m_outputFile);
		}

		void operator()(const Mat &_veloPoints, const Mat &_segImg, const Mat &_refinedImg, Eigen::ArrayXXf &_intersectedPts,
						Mat &_reflectivity, Eigen::MatrixXf &_veloImgPoints, float &_areaDifference, const Mat &_inputImg);

		bool isMode2D() { return m_printOnly2D; }

	protected:

		string m_segRoot;
		string m_refinedRoot;
		string m_refImgPrefix;
		string m_segImgPrefix;
		string m_vizImgPrefix;
		string m_outputFile;
		std::ofstream m_fout;
		bool m_saveVizImg;
		bool m_printOnly2D;
		int m_maxWidth, m_maxLength;
		float m_dThrehold;
		int m_nIter;

		virtual void
		ProcessProjectedLidarPts(const Eigen::MatrixXf &_veloImg, const Mat &_veloPoints, const Mat &_reflectivity,
								 Mat &_inputImg) override;

		void IntersectIn3D(const Eigen::MatrixXf _veloImg, const Mat &_veloPoints, const Mat &_reflectivity,
						   const Mat &_refinedImg, const double &_thresh, Eigen::ArrayXXf &_intersectedImg,
						   Mat &_vizImg);

		void IntersectIn3D(const Eigen::MatrixXf _veloImg, const Mat &_veloPoints, const Mat &_reflectivity,
						   const Mat &_refinedImg, const double &_thresh, Eigen::ArrayXXf &_intersectedPts);

		void PrintToFile(Eigen::ArrayXXf &_intersectedPts);

		double OtsuThresholdRoad(const Eigen::MatrixXf _veloImg, const Mat &_segImg, const Mat &_reflectivity);

		virtual void ParseXML() override;

		void
		ComputeDistanceSum(const Eigen::MatrixXf &_veloImg, const Mat &_veloPoints, const Mat &_segImg,
						   float &distSum);

		static double surfaceModel(double *k, size_t dim, void *params);

		static double planeModel(double *k, size_t dim, void *params);

		double roadSurfArea(double dx, double dy);

		double roadPlaneArea(double dx, double dy);

	};
}

#endif
