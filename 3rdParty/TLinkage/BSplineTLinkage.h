#ifndef B_SPLINE_T_LINKAGE_
#define B_SPLINE_T_LINKAGE_

#include"TLinkage.h"
#include<unordered_map>
#include"interpolation.h"

namespace LD {

	class BSplineTLinkage : public TLinkage {
    public:
        virtual ArrayXf GenerateHypothesis(const vector<ArrayXf> &_samples);

        virtual double Distance(ArrayXf _dataPoint, ArrayXf _model) override;

        virtual void FitModel(const ArrayXXf &_clusters, ArrayXf &_model) override;

        BSplineTLinkage(string _xmlFile);

        ArrayXf ConvertCoeffsTable2Model(alglib::real_2d_array &_yCoeffs, alglib::real_2d_array &_zCoeffs);

        void CreateSplineInterpolants(ArrayXf _model, alglib::spline1dinterpolant &_yOfX,
                                      alglib::spline1dinterpolant &_zOfX);

        virtual void PrintModelsToFile(vector<ArrayXf> _models, const string &_imgName) override;

        virtual void VisualizeModel(ArrayXf &_model, ArrayXXf &_coordinates) override;

        virtual bool IsModelOnRight(const ArrayXf &_model) override;

        virtual void ShiftModelBy(ArrayXf &_model, const float &_shiftBy) override;

    protected:
        void ParseXML() override;

        float m_regularizationConst;
        float m_resolution;
        float m_minX, m_maxX;
        int m_params1dSpline;

	};

}
#endif
