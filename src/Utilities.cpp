#include<fstream>
#include<sstream>
#include"../include/Utilities.h"

namespace LD {

    bool isValid(long long int r, long long int c, long long int rows, long long int cols) {
        return r >= 0 && c >= 0 && r < rows && c < cols;
    }

    void ReadEigenMatFromFile(std::ifstream &_fin, Eigen::ArrayXXf &_data, bool _shouldTranspose) {

        unsigned long long int rows, cols;
        _fin >> rows >> cols;

        _data.resize(rows, cols);
        for (unsigned long long int r = 0; r < rows; r++)
            for (unsigned long long int c = 0; c < cols; c++)
                _fin >> _data(r, c);

        if (_shouldTranspose)
            _data.transposeInPlace();

    }


    void ReadEigenMatFromFile(const string &_fileName, Eigen::ArrayXXf &_data, bool _shouldTranspose) {
        typedef unsigned long long int ulli;
        std::ifstream fin(_fileName);
        if (!fin)
            throw runtime_error("Can't open " + _fileName);
        string line;
        vector<vector<float> > els;
        ulli rows = 0, cols = 0;
        while (std::getline(fin, line)) {
            std::istringstream is(line);
            float el;
            els.push_back(vector<float>());
            while (is >> el)
                els[rows].push_back(el);
            if (!cols)
                cols = els[rows].size();
            else if (cols != els[rows].size())
                throw runtime_error("Number of columns are inconsistent");
            rows++;
        }

        _data.resize(rows, cols);
        for (ulli r = 0; r < rows; r++)
            for (ulli c = 0; c < cols; c++)
                _data(r, c) = els[r][c];

        if (_shouldTranspose)
            _data.transposeInPlace();
    }

    void CreateAlglibArray(const vector<Eigen::ArrayXf> &_samples, vector<alglib::real_1d_array> &_coordinates) {
        if (!_samples.size())
            return;

        _coordinates.resize(_samples[0].size());
        for (int i = 0; i < _coordinates.size(); i++)
            _coordinates[i].setlength(_samples.size());

        for (int i = 0; i < _coordinates.size(); i++)
            for (int j = 0; j < _samples.size(); j++)
                _coordinates[i](j) = _samples[j](i);
    }


    void CreateAlglibArray(const Eigen::ArrayXXf _samples, vector<alglib::real_1d_array> &_coordinates) {
        if (!_samples.cols())
            return;

        _coordinates.resize(_samples.cols());
        for (int i = 0; i < _coordinates.size(); i++)
            _coordinates[i].setlength(_samples.rows());

        for (int i = 0; i < _coordinates.size(); i++)
            for (int j = 0; j < _samples.rows(); j++)
                _coordinates[i](j) = _samples(j, i);
    }

// get the model coeff using SVD deccomposition
    Eigen::MatrixXf computeModel(Eigen::MatrixXf &fitdata) {
        // fitdata: The minimum number of data points to fit for the model
        cv::Mat A(fitdata.rows(), fitdata.rows(), CV_32FC1); // A*x = b, using SVD to get x
        cv::Mat b;
        Eigen::MatrixXf modelcoff;
        for (int i = 0; i < fitdata.rows(); ++i) {
            for (int j = 0; j < fitdata.rows(); ++j) {
                switch (j) {
                    case 0:
                        A.at<float>(i, j) = 1;
                        break;
                    case 1:
                        A.at<float>(i, j) = fitdata(i, 0);
                        break;
                    case 2:
                        A.at<float>(i, j) = fitdata(i, 1);
                        break;
                    case 3:
                        A.at<float>(i, j) = fitdata(i, 0) * fitdata(i, 0);
                        break;
                    case 4:
                        A.at<float>(i, j) = fitdata(i, 0) * fitdata(i, 1);
                        break;
                    case 5:
                        A.at<float>(i, j) = fitdata(i, 1) * fitdata(i, 1);
                        break;
                    case 6:
                        A.at<float>(i, j) = pow(fitdata(i, 0), 3);
                        break;
                    case 7:
                        A.at<float>(i, j) = pow(fitdata(i, 0), 2) * fitdata(i, 1);
                        break;
                    case 8:
                        A.at<float>(i, j) = fitdata(i, 0) * pow(fitdata(i, 1), 2);
                        break;
                    case 9:
                        A.at<float>(i, j) = pow(fitdata(i, 1), 3);
                        break;
                    default:
                        std::cerr << "wrong seg coeff result" << std::endl;
                }
            }

            b.push_back(fitdata(i, 2));
        }
        Eigen::MatrixXf _A;
        cv2eigen(A, _A);
        Eigen::MatrixXf _b;
        cv2eigen(b, _b);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(_A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        modelcoff = svd.solve(_b);
        return modelcoff;
    }


    retresult mcdcomputeres(Eigen::MatrixXf modelcoff, Eigen::MatrixXf lidarpts, float thresdistance, bool status) {
        // modelcoeff : parameters of the model
        // lidarpts : all the lidar 3D points
        retresult res;
        cv::Mat inliers;
        int lrows = lidarpts.rows(), lcols = modelcoff.rows();
        Eigen::MatrixXf inlierroadpts, computeZ = Eigen::MatrixXf::Ones(lrows, lcols);
        if(lrows != 0 && lcols != 0) {
            computeZ.col(1) = lidarpts.col(0);
            computeZ.col(2) = lidarpts.col(1);
            computeZ.col(3) = (lidarpts.col(0)).array().pow(2);
            computeZ.col(4) = (lidarpts.col(0)).array() * (lidarpts.col(1)).array();
            computeZ.col(5) = ((lidarpts.col(1)).array()).pow(2);
            computeZ.col(6) = ((lidarpts.col(0)).array()).pow(3);
            computeZ.col(7) = ((lidarpts.col(0)).array()).pow(2) * (lidarpts.col(1)).array();
            computeZ.col(8) = (lidarpts.col(0)).array() * (lidarpts.col(1)).array().pow(2);
            computeZ.col(9) = ((lidarpts.col(1)).array()).pow(3);
            Eigen::MatrixXf distance = computeZ * modelcoff; //TODO: use vertical distance
            if(status) {
                for (int i = 0; i < distance.rows(); ++i) {
                    if (fabs(distance(i)) <= thresdistance) {
                        cv::Mat inlierpts = (cv::Mat_<float>(1, 4) << lidarpts(i, 0), lidarpts(i, 1), lidarpts(i,
                                                                                                               2), fabs(
                                distance(i)));
                        inliers.push_back(inlierpts);
                    }
                }
            }
            else{
                for (int i = 0; i < distance.rows(); ++i) {
                        cv::Mat inlierpts = (cv::Mat_<float>(1, 4) << lidarpts(i, 0), lidarpts(i, 1), lidarpts(i,
                                                                                                               2), fabs(
                                distance(i)));
                        inliers.push_back(inlierpts);

                }
            }
        }
        cv::cv2eigen(inliers, inlierroadpts);
        res.modelcoff = modelcoff;
        res.inliers = inlierroadpts;
        res.ninliers = inlierroadpts.rows();
        return res;
    }


    retresult
    runRansacRoad(Eigen::MatrixXf &lidarpts, int nSamNumForModel, int maxtrials, int ninlierThreshold,
                  float dThreshold) {
        if (lidarpts.cols() != 3)
            std::cerr << "The input matrix should be 3d..." << std::endl;

        int histcountinliers = 0;
        long long int nIter = maxtrials;
        Eigen::MatrixXf retinliers;
        retresult finalres, retdata;

        for (int count = 0; count < nIter; ++count) {
            if (count > nIter)
                break;

            // 1. sampling
            Eigen::MatrixXi sampleMask = Eigen::ArrayXXi::Zero(1, nSamNumForModel);
            // Takes nSamNumForModel different samples
            if (sampleMask.sum() != nSamNumForModel) {
                Eigen::ArrayXXf ind =
                        lidarpts.rows() / 2 * (Eigen::MatrixXf::Random(1, abs(nSamNumForModel - sampleMask.sum())) +
                                               Eigen::MatrixXf::Constant(1, abs(nSamNumForModel - sampleMask.sum()),
                                                                         1.));
                for (int nidx = 0; nidx < ind.cols(); ++nidx) {
                    sampleMask(0, nidx) = abs(floor(ind(0, nidx)));
                }
            }
            cv::Mat fitdata;
            for (int fitindex = 0; fitindex < nSamNumForModel; ++fitindex) {
                int row_pts = abs(sampleMask(0, fitindex));
                float pts[1][3] = {lidarpts(row_pts, 0), lidarpts(row_pts, 1), lidarpts(row_pts, 2)};
                cv::Mat tempts(1, 3, CV_32FC1, &pts, 2);
                fitdata.push_back(tempts);
            }
            // 2. create the model
            Eigen::MatrixXf fitlidardata;
            cv2eigen(fitdata, fitlidardata);
            Eigen::MatrixXf curModel = computeModel(fitlidardata);
            // 3. model inlier estimation
            retdata = mcdcomputeres(curModel, lidarpts, dThreshold, true);
            //4. Check the size of inliers
            if (retdata.ninliers >= ninlierThreshold) {
                finalres.modelcoff = retdata.modelcoff;
                finalres.inliers = retdata.inliers;
                finalres.ninliers = retdata.ninliers;
                return finalres;
            }
            if (retdata.ninliers > histcountinliers) {
                histcountinliers = retdata.ninliers;
                finalres.modelcoff = retdata.modelcoff;
                finalres.inliers = retdata.inliers;
                finalres.ninliers = retdata.ninliers;
                float p = 0.99;
                float e = 1 - finalres.ninliers / lidarpts.rows();
                nIter = abs(ceil(log(1 - p) / log(1 - pow(1 - e, nSamNumForModel))));
                if (fabs(nIter) > maxtrials || fabs(nIter) < -maxtrials) {
                    nIter = maxtrials;
                }
            }
            if (count >= maxtrials) {
                std::cout << "The maximum trails has been reached" << std::endl;
                break;
            }
        }
        return finalres;
    }

    Eigen::MatrixXf computePlaneModel(Eigen::ArrayXXf fitdata) {
        Eigen::ArrayXf _model;
        Eigen::ArrayXf means = fitdata.colwise().mean();
        Eigen::MatrixXf zeroCentered = (fitdata.rowwise() - means.transpose()).matrix();
        Eigen::EigenSolver<Eigen::MatrixXf> solver(zeroCentered.transpose() * zeroCentered);
        unsigned long long int minEigenValueIndex;
        solver.eigenvalues().real().array().minCoeff(
                &minEigenValueIndex); //eigen values will be real as matrix is symmetric
        _model.resize(4);
        _model.head(3) = solver.eigenvectors().col(minEigenValueIndex).real();
        _model.head(3).matrix().normalize();
        _model(3) = -_model.head(3).matrix().dot(means.matrix());
        return _model.matrix();
    }

    retresult mcdPlaneComput(Eigen::MatrixXf modelcoff, Eigen::MatrixXf lidarpts, float thresdistance, bool status) {
        // modelcoeff : parameters of the model
        // lidarpts : all the lidar 3D points

        cv::Mat inliers;
        Eigen::MatrixXf inlierroadpts;
        lidarpts.conservativeResize(lidarpts.rows(), lidarpts.cols()+1);
        lidarpts.col(lidarpts.cols()-1) = Eigen::VectorXf::Ones(lidarpts.rows(), 1);
        float coeffsum = sqrt(std::pow(modelcoff(0, 0), 2) + std::pow(modelcoff(1, 0), 2) + std::pow(modelcoff(2, 0), 2));
        if(fabs(coeffsum) != 0 && lidarpts.rows()!= 0) {
            Eigen::VectorXf computeZ = (lidarpts * modelcoff).cwiseAbs() / coeffsum;
            Eigen::ArrayXf distance = computeZ.array() + 1.2 * pow(10, -3) * (((lidarpts.col(0)).array()).abs2() +
                                                                              ((lidarpts.col(1)).array()).abs2() +
                                                                              ((lidarpts.col(2)).array()).abs2()).sqrt()
                                                      -0.1 * pow(10, -3);
            if(status) {
                for (int i = 0; i < distance.rows(); ++i) {
                    if (distance(i, 0) <= thresdistance) {
                        cv::Mat inlierpts = (cv::Mat_<float>(1, 4) << lidarpts(i, 0), lidarpts(i, 1), lidarpts(i,
                                                                                                               2), distance(
                                i, 0));
                        inliers.push_back(inlierpts);
                    }
                }
            }
            else{
                for (int i = 0; i < distance.rows(); ++i) {
                        cv::Mat inlierpts = (cv::Mat_<float>(1, 4) << lidarpts(i, 0), lidarpts(i, 1), lidarpts(i,
                                                                                                               2), distance(
                                i, 0));
                        inliers.push_back(inlierpts);

                }
            }
        }
        cv::cv2eigen(inliers, inlierroadpts);
        return {modelcoff, inlierroadpts, inlierroadpts.rows()};
    }


    // fit the plane using ransac
    retresult
    runRansacplanefit(Eigen::MatrixXf &lidarpts, int nSamNumForModel, int maxtrials, int ninlierThreshold,
                      float dThreshold) {

        if (lidarpts.cols() != 3)
            std::cerr << "The input matrix should be 3d..." << std::endl;

        int histcountinliers = 0;
        long long int nIter = maxtrials;
        Eigen::MatrixXf retinliers;
        retresult finalres, retdata;

        for (int count = 0; count < nIter; ++count) {
            // 1. sampling
            Eigen::MatrixXi sampleMask = Eigen::ArrayXXi::Zero(1, nSamNumForModel);
            // Takes nSamNumForModel different samples
            if (sampleMask.sum() != nSamNumForModel) {
                Eigen::ArrayXXf ind =
                        lidarpts.rows() / 2 * (Eigen::MatrixXf::Random(1, abs(nSamNumForModel - sampleMask.sum())) +
                                               Eigen::MatrixXf::Constant(1, abs(nSamNumForModel - sampleMask.sum()),
                                                                         1.));
                for (int nidx = 0; nidx < ind.cols(); ++nidx) {
                    sampleMask(0, nidx) = abs(floor(ind(0, nidx)));
                }
            }
            cv::Mat fitdata;
            for (int fitindex = 0; fitindex < nSamNumForModel; ++fitindex) {
                int row_pts = abs(sampleMask(0, fitindex));
                float pts[1][3] = {lidarpts(row_pts, 0), lidarpts(row_pts, 1), lidarpts(row_pts, 2)};
                cv::Mat tempts(1, 3, CV_32FC1, &pts, 2);
                fitdata.push_back(tempts);
            }
            Eigen::MatrixXf fitlidardata;
            cv2eigen(fitdata, fitlidardata);
            Eigen::MatrixXf curModel = computePlaneModel(fitlidardata.array()); // 2. create the model
            retdata = mcdPlaneComput(curModel, lidarpts, dThreshold, true); // 3. model inlier estimation
            //4. Check the size of inliers
            if (retdata.ninliers >= ninlierThreshold) {
                finalres.modelcoff = retdata.modelcoff;
                finalres.inliers = retdata.inliers;
                finalres.ninliers = retdata.ninliers;
                return finalres;
            }

            if (retdata.ninliers >= histcountinliers) {
                histcountinliers = retdata.ninliers;
                finalres.modelcoff = retdata.modelcoff;
                finalres.inliers = retdata.inliers;
                finalres.ninliers = retdata.ninliers;
                float p = 0.99;
                float e = 1 - finalres.ninliers / lidarpts.rows();
                nIter = abs(ceil(log(1 - p) / log(1 - pow(1 - e, nSamNumForModel))));
                if (fabs(nIter) > maxtrials || fabs(nIter) < -maxtrials) {
                    nIter = maxtrials;
                }
            }

            if (count >= maxtrials || count >= nIter) {
                std::cout << "The maximum trails has been reached" << std::endl;
                break;
            }
        }
        return finalres;
    }

}