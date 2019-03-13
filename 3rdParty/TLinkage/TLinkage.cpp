#include "TLinkage.h"
#include"UnionFind.h"
#include<cstdlib>
#include<limits>
#include<fstream>
#include <boost/algorithm/string/predicate.hpp>
#include"Utilities.h"

//Samplers
#include"UniformSampler.h"
#include"DistBasedSampler.h"

//Outlier Rejectors
#include"MaxDiffOR.h"
#include"KLargestOR.h"

//Preference finders
#include"ExpPreferenceFinder.h"
#include"HardPreferenceFinder.h"
#include"GaussPreferenceFinder.h"
#include"TukeyPreferenceFinder.h"

namespace LD {

    TLinkage::TLinkage(int _minSamples, int _modelParams, string _xmlFile) :
            Solver(_xmlFile), m_minSamples(_minSamples), m_modelParams(_modelParams),
            m_sampler(std::make_unique<UniformSampler>(m_xmlFileName)),
            m_outlierRejector(std::make_unique<MaxDiffOR>(_minSamples, m_xmlFileName)) {
        ParseXML();
        m_foutModels.open(m_modelFile);
        m_foutClusters.open(m_clusterFile);

    }

    void TLinkage::ParseXML() {
        m_xml = m_xml.child("TLinkage");

        pugi::xml_node solverInstanceNode = m_xml.child("SolverInstance");

        string sampler = solverInstanceNode.attribute("sampler").as_string();
        string preferenceFinder = solverInstanceNode.attribute("preferenceFinder").as_string();
        string outlierRejector = solverInstanceNode.attribute("outlierRejector").as_string();
        m_samplesPerDataPt = solverInstanceNode.attribute("samplesPerDataPt").as_int();
        m_dataFile = solverInstanceNode.attribute("dataFile").as_string();
        m_shouldTranspose = solverInstanceNode.attribute("shouldTranspose").as_bool();
        m_modelFile = solverInstanceNode.attribute("modelFile").as_string();
        m_saveClusters = solverInstanceNode.attribute("saveClusters").as_bool(true);
        m_clusterFile = solverInstanceNode.attribute("clusterFile").as_string();
        m_minShift = solverInstanceNode.attribute("minShift").as_float();
        m_maxShift = solverInstanceNode.attribute("maxShift").as_float();
        m_errorThreshold = solverInstanceNode.attribute("errorThreshold").as_float();
        m_shiftIncrement = solverInstanceNode.attribute("shiftIncrement").as_float();

        if (sampler.empty() || preferenceFinder.empty() || outlierRejector.empty() || m_dataFile.empty() ||
            !m_samplesPerDataPt || m_modelFile.empty() || (m_saveClusters && m_clusterFile.empty()) || !m_minShift ||
            !m_maxShift || !m_errorThreshold || !m_shiftIncrement)
            throw runtime_error(
                    "TLinkage SolverInstance node doesn't have one or more of the following attributes: sampler, preferenceFinder, outlierRejector, dataFile, samplesPerDataPt, modelFile, saveClusters, clusterFile, minShift, maxShift, errorThreshold, shiftIncrement");

        //Set Sampler
        if (boost::iequals(sampler, "Uniform"))
            SetSampler(SamplingMethod::UNIFORM);
        else if (boost::iequals(sampler, "PreferNear"))
            SetSampler(SamplingMethod::PREFER_NEAR);
        else
            throw runtime_error("No such sampler found: " + sampler);

        //Set Outlier Rejector
        if (boost::iequals(outlierRejector, "MaxSizeChange"))
            SetOutlierRejector(OutlierRejectionMethod::MAX_SIZE_CHANGE);
        else if (boost::iequals(outlierRejector, "KLargest"))
            SetOutlierRejector(OutlierRejectionMethod::K_LARGEST);
        else if (boost::iequals(outlierRejector, "none"))
            SetOutlierRejector(OutlierRejectionMethod::NONE);
        else
            throw runtime_error("No such outlier rejector found: " + outlierRejector);

        //Set Preference Finder
        if (boost::iequals(preferenceFinder, "Exp"))
            SetPreferenceFinder(PreferenceMethod::EXP);
        else if (boost::iequals(preferenceFinder, "Gauss"))
            SetPreferenceFinder(PreferenceMethod::GAUSS);
        else if (boost::iequals(preferenceFinder, "Tukey"))
            SetPreferenceFinder(PreferenceMethod::TUKEY);
        else if (boost::iequals(preferenceFinder, "Hard"))
            SetPreferenceFinder(PreferenceMethod::HARD);
        else
            throw runtime_error("No such preference finder found: " + preferenceFinder);

    }

    void TLinkage::operator()(const ArrayXXf &_data, ArrayXf &_clusters, vector<ArrayXf> &_models) {
        if (m_debug)
            cout << "Entering TLinkage::()" << endl;

        ArrayXXf sampleIndices, hypotheses, residuals, pref;
        std::unordered_map<int, ulli> clusterID2Index;
        std::unordered_map<int, vector<ulli> > clusterID2PtIndices;
        Sample(_data, sampleIndices, m_samplesPerDataPt * _data.cols());
        GenerateHypotheses(_data, sampleIndices, hypotheses);
        FindResiduals(_data, hypotheses, residuals);
        FindPreferences(residuals, pref);
        Cluster(pref, _clusters, clusterID2PtIndices);
        FitModels(_data, _clusters, _models, clusterID2Index);
        //FIXME: add condition
        FindParallelModels(_models, _data, _clusters, clusterID2Index, clusterID2PtIndices, _models, _clusters);

        if(m_debug)
            PrintModelsToFile(_models, m_imgName);

        if (m_debug)
            cout << "Exiting TLinkage::()" << endl;
    }

    void TLinkage::Run() {
        if (m_debug)
            cout << "Entering TLinkage::Run()" << endl;

        std::ifstream fin(m_dataFile.c_str());
        if (!fin)
            throw runtime_error("couldn't open " + m_dataFile);
        while (fin >> m_imgName) {
            ArrayXXf data;
            ArrayXf clusters;
            vector<ArrayXf> models;
            ReadEigenMatFromFile(fin, data, m_shouldTranspose);
            if (data.cols() > m_minSamples) {
                this->operator()(data, clusters, models);
                if (m_saveClusters)
                    PrintClustersToFile(clusters, m_imgName);
                //PrintModelsToFile(models, m_imgName);
            }
        }

        if (m_debug)
            cout << "Exiting TLinkage::Run()" << endl;
    }

    void TLinkage::PrintClustersToFile(const ArrayXf &_clusters, const string &_imgName) {
        if (m_debug)
            cout << "Exiting TLinkage::PrintClustersToFile()" << endl;

        m_foutClusters << _imgName << endl;
        m_foutClusters << _clusters.size() << endl;
        m_foutClusters << _clusters << endl;

        if (m_debug)
            cout << "Exiting TLinkage::PrintClustersToFile()" << endl;
    }

    void TLinkage::PrintModelsToFile(vector<ArrayXf> _models, const string &_imgName) {
        if (m_debug)
            cout << "Entering TLinkage::PrintModelsToFile()" << endl;

        m_foutModels << _imgName << endl;
        m_foutModels << _models.size() << "\t" << m_modelParams << endl;
        for (int i = 0; i < _models.size(); i++)
            m_foutModels << _models[i] << endl << endl;

        if (m_debug)
            cout << "Exiting TLinkage::PrintModelsToFile()" << endl;
    }

    void TLinkage::SetSampler(SamplingMethod _method) {
        switch (_method) {
            case SamplingMethod::PREFER_NEAR:
                m_sampler = std::make_unique<DistBasedSampler>(m_xmlFileName);
                break;
            case SamplingMethod::UNIFORM:
                m_sampler = std::make_unique<UniformSampler>(m_xmlFileName);
                break;
            default:
                throw runtime_error("Attempt to set unknown sampler");

        }
    }

    void TLinkage::SetOutlierRejector(OutlierRejectionMethod _method) {
        switch (_method) {
            case OutlierRejectionMethod::MAX_SIZE_CHANGE:
                m_outlierRejector = std::make_unique<MaxDiffOR>(m_minSamples, m_xmlFileName);
                break;
            case OutlierRejectionMethod::K_LARGEST:
                m_outlierRejector = std::make_unique<KLargestOR>(m_xmlFileName);
                break;
            case OutlierRejectionMethod::NONE:
                m_outlierRejector = nullptr;
                break;
            default:
                throw runtime_error("Attempt to set unknown outlier rejector");
        }
    }

    void TLinkage::SetPreferenceFinder(PreferenceMethod _method) {

        switch (_method) {
            case PreferenceMethod::EXP:
                m_preferenceFinder = std::make_unique<ExpPreferenceFinder>(m_xmlFileName);
                break;
            case PreferenceMethod::GAUSS:
                m_preferenceFinder = std::make_unique<GaussPreferenceFinder>(m_xmlFileName);
                break;
            case PreferenceMethod::HARD:
                m_preferenceFinder = std::make_unique<HardPreferenceFinder>(m_xmlFileName);
                break;
            case PreferenceMethod::TUKEY:
                m_preferenceFinder = std::make_unique<TukeyPreferenceFinder>(m_xmlFileName);
                break;
            default:
                throw runtime_error("Preference method not available");
        }

    }

    void TLinkage::Sample(const ArrayXXf &_data, ArrayXXf &_sampleIndices,
                          const unsigned long long int &_numSamples) {

        if (m_debug)
            cout << "Entering TLinkage::Sample()" << endl;

        if (!m_sampler)
            throw runtime_error("Sampler not set");

        (*m_sampler)(_data, _sampleIndices, _numSamples, m_minSamples);

        if (m_debug)
            cout << "Exiting TLinkage::Sample()" << endl;
    }

    void TLinkage::FindResiduals(const ArrayXXf &_data, const ArrayXXf &_hypotheses,
                                 ArrayXXf &_residuals) {

        if (m_debug)
            cout << "Entering TLinkage::FindResiduals()" << endl;

        _residuals = ArrayXXf(_hypotheses.cols(), _data.cols());  //column major memory considered
#pragma omp parallel for  collapse(2)
        for (ulli i = 0; i < _residuals.rows(); i++)
            for (ulli j = 0; j < _residuals.cols(); j++)
                _residuals(i, j) = Distance(_data.col(j), _hypotheses.col(i));

        if (m_debug)
            cout << "Exiting TLinkage::FindResiduals()" << endl;
    }

    void TLinkage::FindPreferences(const ArrayXXf &_residuals, ArrayXXf &_preferences) {

        if (m_debug)
            cout << "Entering TLinkage::FindPreferences()" << endl;

        (*m_preferenceFinder)(_residuals, _preferences);

        if (m_debug)
            cout << "Exiting TLinkage::FindPreferences()" << endl;

    }

    void TLinkage::GenerateHypotheses(const ArrayXXf &_data,
                                      const ArrayXXf &_sampleIndices, ArrayXXf &_hypotheses) {

        if (m_debug)
            cout << "Entering TLinkage::GenerateHypotheses()" << endl;

        _hypotheses.resize(m_modelParams, _sampleIndices.cols());
        vector<ArrayXf> samples(m_minSamples);

        for (ulli c = 0; c < _sampleIndices.cols(); c++) {
            for (ulli r = 0; r < _sampleIndices.rows(); r++)
                samples[r] = _data.col(_sampleIndices(r, c)); //FIXME: Deep Copy
            _hypotheses.col(c) = GenerateHypothesis(samples);

        }

        if (m_debug)
            cout << "Exiting TLinkage::GenerateHypotheses()" << endl;

    }

    void TLinkage::Cluster(ArrayXXf &_preferences, ArrayXf &_clusters,
                           std::unordered_map<int, vector<ulli> > &_clusterID2PtIndices, const int &_noiseIndex) {
        if (m_debug)
            cout << "Entering TLinkage::Cluster()" << endl;

        ArrayXXf distances;
        CalculateTanimotoDist(_preferences, distances);

        _clusters = ArrayXf(_preferences.cols());

        UnionFind uf(_preferences.cols());

        //find min dist
        ulli pt1, pt2;
        double minDist = distances.minCoeff(&pt1, &pt2);
        while (minDist < 1) {  //FIXME: is it correct to stop the first time you see 1?

            //remove other point
            if (pt2 > 0)
                distances.col(pt2).head(pt2) = std::numeric_limits<float>::max();
            distances.row(pt2).tail(distances.cols() - pt2) = std::numeric_limits<float>::max();

            //modify the preferences of the retained point so it expresses preferences of the union
            _preferences.col(pt1) = _preferences.col(pt1).min(_preferences.col(pt2));

            //update the distances matrix
            for (ulli i = 0; i < pt1; i++)
                if (distances(i, pt1) != std::numeric_limits<float>::max())
                    distances(i, pt1) = Tanimoto(_preferences.col(pt1), _preferences.col(i));

            for (ulli i = pt1 + 1; i < distances.cols(); i++)
                if (distances(pt1, i) != std::numeric_limits<float>::max())
                    distances(pt1, i) = Tanimoto(_preferences.col(pt1), _preferences.col(i));
            uf.unionSet(pt1, pt2);
            minDist = distances.minCoeff(&pt1, &pt2);
        }

        for (ulli i = 0; i < _clusters.size(); i++)
            _clusterID2PtIndices[uf.findSet(i)].push_back(i);

        //Remove small, useless clusters

        for (auto it = _clusterID2PtIndices.begin(); it != _clusterID2PtIndices.end();) {
            if (it->second.size() > m_minSamples) {
                for (auto &index : it->second)
                    _clusters(index) = it->first;
                it++;
            } else {
                for (auto &index : it->second)
                    _clusters(index) = _noiseIndex;
                it = _clusterID2PtIndices.erase(it);
            }
        }

        if (m_debug)
            cout << "Exiting TLinkage::Cluster()" << endl;
    }

    void TLinkage::CalculateTanimotoDist(const ArrayXXf &_preferences, ArrayXXf &_distances) {
        if (m_debug)
            cout << "Entering TLinkage::CalculateTanimotoDist()" << endl;
        _distances = ArrayXXf(_preferences.cols(), _preferences.cols());
        //create symmetric matrix without storing two copies
        for (ulli r = 0; r < _distances.rows(); r++) {
            for (ulli c = 0; c < _distances.cols(); c++) {
                if (c <= r)
                    _distances(r, c) = std::numeric_limits<float>::max(); //so it's not min ever
                else
                    _distances(r, c) = Tanimoto(_preferences.col(r), _preferences.col(c));
            }
        }

        if (m_debug)
            cout << "Exiting TLinkage::CalculateTanimotoDist()" << endl;
    }

    float TLinkage::Tanimoto(const ArrayXf &_a, const ArrayXf &_b) {
        float dotProd = _a.matrix().dot(_b.matrix());
        return 1 - dotProd / (_a.matrix().squaredNorm() + _b.matrix().squaredNorm() - dotProd);
    }

    void TLinkage::RejectOutliers(const ArrayXf &_clusters,
                                  const std::unordered_map<int, vector<ulli> > &_clusterID2PtIndices, ArrayXf &_out,
                                  const int &_noiseIndex) {

        if (m_debug)
            cout << "Entering TLinkage::RejectOutliers()" << endl;

        if (!m_outlierRejector)
            return;

        (*m_outlierRejector)(_clusters, _clusterID2PtIndices, _out, _noiseIndex);

        if (m_debug)
            cout << "Exiting TLinkage::RejectOutliers()" << endl;
    }

    void TLinkage::FitModels(const ArrayXXf &_data, const ArrayXf &_clusters, vector<ArrayXf> &_models,
                             std::unordered_map<int, ulli> &_clusterID2Index, const int &_noiseIndex) {
        if (m_debug)
            cout << "Entering TLinkage::FitModels()" << endl;

        vector<vector<float> > clusteredPoints;

        for (ulli i = 0; i < _clusters.size(); i++) {
            if (_clusters(i) != _noiseIndex) {
                if (_clusterID2Index.find(_clusters(i)) == _clusterID2Index.end()) {
                    _clusterID2Index[_clusters(i)] = clusteredPoints.size();
                    clusteredPoints.push_back(vector<float>());
                }
                for (ulli j = 0; j < _data.rows(); j++)
                    clusteredPoints[_clusterID2Index[_clusters(i)]].push_back(_data(j, i));
            }
        }

        if (m_debug)
            cout << "Clubbed points of the same cluster" << endl;

        vector<ArrayXXf> clusters(clusteredPoints.size());

        for (ulli i = 0; i < clusters.size(); i++) {
            clusters[i] = Map<Matrix<float, Dynamic, Dynamic, RowMajor>, Unaligned, OuterStride<> >(
                    clusteredPoints[i].data(), clusteredPoints[i].size() / _data.rows(), _data.rows(),
                    OuterStride<>(_data.rows())).array(); //this is a deep copy
            _models.push_back(ArrayXf());
            FitModel(clusters[i], _models.back());
        }

        if (m_debug)
            cout << "Exiting TLinkage::FitModels()" << endl;
    }

    void TLinkage::FindParallelModels(const vector<ArrayXf> &_models, const ArrayXXf &_data, const ArrayXf &_clusters,
                                      const std::unordered_map<int, ulli> &_clusterID2Index,
                                      const std::unordered_map<int, vector<ulli> > &_clusterID2PtIndices,
                                      vector<ArrayXf> &_refinedModels, ArrayXf &_refinedClusters,
                                      const int &_noiseIndex) {

        if (m_debug)
            cout << "Entering TLinkage::FindParallelModels()" << endl;

        //Find total number of clusters
/*        if (_models.size() > 2 && m_debug)
            cout << "-----------------WARNING: Too many clusters: " << _clusterID2PtIndices.size()
                 << "----------------------------" << endl;*/

        //Find largest model

        vector<std::pair<int, ulli> > count;
        vector<ArrayXf> refModelsCopy; //this is necessary if _models == _refinedModels

        for (auto it = _clusterID2PtIndices.begin(); it != _clusterID2PtIndices.end(); it++)
            count.push_back(std::make_pair(it->first, it->second.size()));

        int largestClusterID;
        int largestSize = 0;
        for (int i = 0; i < count.size(); i++)
            if (count[i].second > largestSize && count[i].first != _noiseIndex)
                largestSize = count[i].second, largestClusterID = count[i].first;

        ArrayXf originalModel = _models[_clusterID2Index.find(largestClusterID)->second];
        refModelsCopy.push_back(originalModel);
        int largestClusterIDOpp;

        if (_models.size() > 1) {

            //Find largest model on opposite side
            bool isOriginalModelOnRight = IsModelOnRight(originalModel);
            largestSize = 0;

            for (int i = 0; i < count.size(); i++)
                if (count[i].second > largestSize &&
                    IsModelOnRight(_models[_clusterID2Index.find(count[i].first)->second]) != isOriginalModelOnRight &&
                    count[i].first != _noiseIndex)
                    largestSize = count[i].second, largestClusterIDOpp = count[i].first;

            if (largestSize > 0) {
                ArrayXf shiftedModel;
                ShiftModel(originalModel, _clusterID2PtIndices.find(largestClusterIDOpp)->second, _data,
                           isOriginalModelOnRight, shiftedModel);
                refModelsCopy.push_back(shiftedModel);
            } else {
                if(m_debug)
                cout << "-----------------WARNING: No cluster on the opposite side ----------------------------"
                     << endl;
            }

        } else{
            if(m_debug)
            cout << "-----------------WARNING: Only 1 cluster found ----------------------------" << endl;}

        _refinedModels = refModelsCopy;
        _refinedClusters = _clusters;

        for (int i = 0; i < _refinedClusters.size(); i++) {
            if (_refinedClusters(i) != largestClusterID && _refinedClusters(i) != largestClusterIDOpp)
                _refinedClusters(i) = _noiseIndex;
        }

        if (m_debug)
            cout << "Exiting TLinkage::FindParallelModels()" << endl;

    }

    void
    TLinkage::ShiftModel(const ArrayXf &_originalModel, const vector<ulli> &_clusteredIndices, const ArrayXXf &_data,
                         bool _isOriginalModelOnRight, ArrayXf &_shiftedModel) {

        if (m_debug)
            cout << "Entering TLinkage::Shiftmodel()" << endl;

        ArrayXf shiftedModelCopy = _originalModel;

        //start shifting it to the right/left

        float minShift = (_isOriginalModelOnRight ? -m_minShift : m_minShift);
        ShiftModelBy(shiftedModelCopy, minShift);
        float error = EvaluateModel(shiftedModelCopy, _clusteredIndices, _data);
        float minError = error;
        float shiftBy = _isOriginalModelOnRight ? -m_shiftIncrement : m_shiftIncrement;
        float totalShift = minShift;
        while (std::abs(totalShift) < m_maxShift) {
            ShiftModelBy(shiftedModelCopy, shiftBy);

            totalShift += shiftBy;
            error = EvaluateModel(shiftedModelCopy, _clusteredIndices, _data);
            if (error < minError) {
                minError = error;
                minShift = totalShift;
            }
        }

        _shiftedModel = _originalModel;
        ShiftModelBy(_shiftedModel, minShift);

        if (minError > m_errorThreshold && m_debug)
            cout << "-----------------WARNING: Error too big " << minError << "----------------------------" << endl;

        if (m_debug)
            cout << "Exiting TLinkage::Shiftmodel()" << endl;

    }

    float TLinkage::EvaluateModel(const ArrayXf &_model, const vector<ulli> &_clusterIndices, const ArrayXXf &_data) {
        float distance = 0;
#pragma omp parallel for
        for (int i = 0; i < _clusterIndices.size(); i++)
            distance += Distance(_data.col(_clusterIndices[i]), _model);
        return distance / _clusterIndices.size();
    }

}