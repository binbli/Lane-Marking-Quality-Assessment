#include"../include/KPercentExtractor.h"

namespace LD {

    void KPercentExtractor::ParseXML() {
        m_xml = m_xml.child("KPercentExtractor");
        m_k = m_xml.attribute("k").as_int();

        if (!m_k)
            throw runtime_error("at least one of the following attribute is missing: k");
    }


    void KPercentExtractor::runDFS(cv::Mat &_noisefreeImg, int x, int y, int &_nclusters, vector<cv::Point> &_pts) {
        _noisefreeImg.at<uchar>(x, y) = 0;

        if (x >= 1 && _noisefreeImg.at<uchar>(x - 1, y) > 0) {
            Point temp(x - 1, y);
            _pts.push_back(temp);
            runDFS(_noisefreeImg, x - 1, y, _nclusters, _pts);
            _nclusters += 1;
        }
        if (x < _noisefreeImg.rows - 1 && _noisefreeImg.at<uchar>(x + 1, y) > 0) {
            Point temp(x + 1, y);
            _pts.push_back(temp);
            runDFS(_noisefreeImg, x + 1, y, _nclusters, _pts);
            _nclusters += 1;
        }
        if (y >= 1 && _noisefreeImg.at<uchar>(x, y - 1) > 0) {
            Point temp(x, y - 1);
            _pts.push_back(temp);
            runDFS(_noisefreeImg, x, y - 1, _nclusters, _pts);
            _nclusters += 1;
        }

        if (y < _noisefreeImg.cols - 1 && _noisefreeImg.at<uchar>(x, y + 1) > 0) {
            Point temp(x, y + 1);
            _pts.push_back(temp);
            runDFS(_noisefreeImg, x, y + 1, _nclusters, _pts);
            _nclusters += 1;
        }

    }

    int KPercentExtractor::noiseRemover(cv::Mat &_segImg, int maxclusters) {

        if (m_debug)
            cout << "Entering Refiner::noiseRemover()" << endl;

        if (_segImg.rows == 0 || _segImg.cols == 0)
            return 0;

        int res = 0, nClusters = 0;
        cv::Mat _noisefreeImg = _segImg.clone();
        vector<cv::Point> pts;
        uint8_t *_segData = _segImg.data;
        int width = _segImg.cols;
        int height = _segImg.rows;
        int _stride = _segImg.step;//in case cols != strides

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (_segImg.at<uchar>(i, j)) {
                    ++res;
                    runDFS(_noisefreeImg, i, j, nClusters, pts);
                    if (nClusters > 0 && nClusters < maxclusters && !pts.empty()) { // remove nClusters less then 100 pixels
                        for (auto &s : pts)
                            _segImg.at<uchar>(s.x, s.y) = 0;
                    }
                    nClusters = 0;
                    pts.clear();
                }
            }
        }

        if (m_debug)
            cout << "Exiting Refiner::noiseRemover()" << endl;

        return res;
    }


    void KPercentExtractor::Preprocess(const Mat &_original, const Mat &_segImg, Mat &_preprocessed) {
        if (m_debug)
            cout << "Entering KPercentExtractor::Preprocess()" << endl;

        cvtColor(_original, _preprocessed, COLOR_RGB2GRAY);
        bitwise_and(_preprocessed, _segImg, _preprocessed);
        noiseRemover(_preprocessed, 100);

        if (m_debug)
            cout << "Exiting KPercentExtractor::Preprocess()" << endl;
    }

    int
    KPercentExtractor::VPtsdector(const cv::Mat &_extractedImg, cv::Mat &_refinedImg, cv::Mat &_labeledImg,
                                  int thresh) {

        if (m_debug)
            cout << "Entering KPercentExtractor::VPtsdector()" << endl;

        if (!_extractedImg.isContinuous())
            std::cerr << "image in VPtsDetector is not conitnuous..." << std::endl;

        if (!_refinedImg.isContinuous())
            std::cerr << "Lane marking in VPtsDetector is not conitnuous..." << std::endl;

        Eigen::MatrixXf A, B;
        std::vector<std::vector<int> > points;
        cv::Mat equalizedImg;
        //equalizing histogram
        if (_extractedImg.channels() == 3) {
            cvtColor(_extractedImg, equalizedImg, CV_BGR2YUV);
            vector<cv::Mat> channels;
            split(equalizedImg, channels);
            equalizeHist(channels[0], channels[0]);
            merge(channels, equalizedImg);
            cvtColor(equalizedImg, equalizedImg, CV_YUV2BGR);
            cvtColor(equalizedImg, equalizedImg, CV_BGR2GRAY);
        } else equalizedImg = _extractedImg.clone();

        minlength = _extractedImg.cols * _extractedImg.cols * 0.001; // define minimum length requirement for any line

        initVPs(equalizedImg, points); //initialize the line segment matrix in format y = m*x + c
        makeLines(A, B, points); //draw lines on image and display
        int status = GetImgVP(equalizedImg, _refinedImg, _labeledImg, A, B, thresh,
                              points); //approximate vanishing point

        if (m_debug)
            cout << "Exist KPercentExtractor::VPtsdector()" << endl;

        return status;
    }

    ntuple_list KPercentExtractor::callLsd(cv::Mat *src) {

        if (m_debug)
            cout << "Entering KPercentExtractor::callLsd()" << endl;

        cv::Mat *src_gray;
        if (src->channels() == 3) {
            unsigned int width = src->size().width;
            unsigned int height = src->size().height;
            src_gray = new cv::Mat(width, height, CV_8UC1);
            // CV_RGB2GRAY: convert RGB image to grayscale
            cvtColor(*src, *src_gray, CV_RGB2GRAY);
        } else
            src_gray = src;

        image_double image; //image_double is a struct defined in 'lsd.h'
        ntuple_list lsd_out;
        unsigned int w = src->size().width;
        unsigned int h = src->size().height;
        image = new_image_double(w, h);
        unsigned char s = 0;//to get image values

        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                s = src_gray->at<uchar>(y, x);
                image->data[x + y * image->xsize] = s;
            }
        }

        lsd_out = lsd(image);
        free_image_double(image);

        if (src->channels() == 3) delete src_gray;

        if (m_debug)
            cout << "Exist KPercentExtractor::callLsd()" << endl;

        return lsd_out;
    }

    void KPercentExtractor::initVPs(cv::Mat grayImag, std::vector<std::vector<int> > &points) {

        if (m_debug)
            cout << "Entering KPercentExtractor::initVPs()" << endl;

        ntuple_list lsdOut;
        lsdOut = callLsd(&grayImag); // use LSD method
        int dim = lsdOut->dim;
        double a, b, c, d;

        for (int i = 0; i < lsdOut->size; i++) // store LSD output to lineSegments
        {
            a = lsdOut->values[i * dim];     // x1
            b = lsdOut->values[i * dim + 1]; // y1
            c = lsdOut->values[i * dim + 2]; // x2
            d = lsdOut->values[i * dim + 3]; // y2

            if (fabs(a - c) < 10 || fabs(b - d) < 10)
                continue;

            if (((a - c) * (a - c) + (b - b) * (b - d)) < minlength)
                continue;

            std::vector<int> temp = {static_cast<int>(a), static_cast<int>(b), static_cast<int>(c),
                                     static_cast<int>(d)};
            points.push_back(temp);
        }

        if (m_debug)
            cout << "Exist KPercentExtractor::initVPs()" << endl;

    };


    void KPercentExtractor::makeLines(Eigen::MatrixXf &A, Eigen::MatrixXf &B, std::vector<std::vector<int> > points) {

        if (m_debug)
            cout << "Entering KPercentExtractor::makeLines()" << endl;

        A = Eigen::MatrixXf::Zero(points.size(), 2);
        B = Eigen::MatrixXf::Zero(points.size(), 1);
        for (int i = 0; i < points.size(); i++) {
            A(i, 0) = -(points[i][3] - points[i][1]);
            A(i, 1) = (points[i][2] - points[i][0]);
            B(i, 0) = A(i, 0) * points[i][0] + A(i, 1) * points[i][1];
        }

        if (m_debug)
            cout << "Exist KPercentExtractor::makeLines()" << endl;
    }

//estimate the vanishing point
    int
    KPercentExtractor::GetImgVP(const cv::Mat _grayimg, cv::Mat &_refinedImg, cv::Mat &labeledImg, Eigen::MatrixXf &A,
                                Eigen::MatrixXf &B, int thresh, std::vector<std::vector<int> > points) {

        if (m_debug)
            cout << "Enter KPercentExtractor::GetImgVP()" << endl;

        std::vector<std::vector<int> > VPlines;
        soln.setZero(); //initialize the solu vec

        for (int i = 0; i < points.size(); i++) {
            for (int j = 0; j < points.size(); j++) {
                if (i >= j)
                    continue;
                Eigen::Matrix2f Atemp;
                Atemp << A(i, 0), A(i, 1),
                        A(j, 0), A(j, 1);

                Eigen::Vector2f btemp;
                btemp << B(i, 0),
                        B(j, 0);

                Eigen::FullPivLU<Eigen::Matrix2f> LV_decomp(Atemp);
                if (static_cast<int> (LV_decomp.rank()) != 2)
                    continue;

                Eigen::Vector2f res = Atemp.colPivHouseholderQr().solve(btemp);
                if (res.rows() == 0 || res.cols() == 0)
                    continue;

                double temperr = (A * res - B).norm() / pow(10, 9);
                minerr = std::numeric_limits<double>::max();

                if (minerr > temperr && res(0, 0) > 0 && res(0, 0) < _grayimg.cols && res(1, 0) > 0 &&
                    res(1, 0) < _grayimg.rows) {
                    minerr = temperr;
                    soln = res;

                    // store the intersection line segments in VPlines
                    std::vector<int> vplinei = {points[i][0], points[i][1], points[i][2], points[i][3]};
                    std::vector<int> vplinej = {points[j][0], points[j][1], points[j][2], points[j][3]};
                    VPlines.push_back(vplinei);
                    VPlines.push_back(vplinej);
                }
            }
        }

        if (!soln.isZero(0)) {

            Eigen::MatrixXf newA;
            Eigen::MatrixXf newB;
            makeLines(newA, newB, VPlines);

            labeledImg = cv::Mat::zeros(_refinedImg.rows, _refinedImg.cols, CV_8UC1);

            for (int k = 0; k < newA.rows(); k++) {
                Eigen::RowVector2f lineqn = newA.row(k);
                float intersection = newB(k, 0);
                for (int i = 0; i < _refinedImg.rows; i++) {
                    for (int j = 0; j < _refinedImg.cols; j++) {
                        Eigen::Vector2f pixels;
                        pixels << j,
                                i;
                        if (_refinedImg.at<uchar>(i, j) >= thresh &&
                            fabs(lineqn * pixels - intersection) / lineqn.norm() < 12) {
                            labeledImg.at<uchar>(i, j) = _refinedImg.at<uchar>(i, j);
                        }
                    }
                }
            }

            if (labeledImg.channels() == 3)
                cv::cvtColor(_refinedImg, _refinedImg, CV_BGR2GRAY);

            if (!labeledImg.isContinuous())
                std::cerr << "_refinedImg in VPtsDetector is not conitnuous..." << std::endl;

            if (m_debug)
                cout << "Exist KPercentExtractor::GetImgVP()" << endl;

            return 1;

        } else
            return 0;

    }


    void KPercentExtractor::LMsfromImg(const Mat &_original, const Mat &_segImg, Mat &_refinedImg) {

        if (m_debug)
            cout << "Entering KPercentExtractor::LMsfromImg()" << endl;

        Mat _extractedImg, _labeledImg;
        Preprocess(_original, _segImg, _extractedImg);


        Mat flattened = _extractedImg.reshape(1, 1).clone();
        if (flattened.isContinuous()) {
            std::sort(flattened.data, flattened.data + flattened.total());
            int numZeros = std::distance(flattened.datastart,
                                         std::upper_bound(flattened.datastart, flattened.dataend, 0));
            int threshIndex = ((flattened.total() - numZeros) * (100 - m_k)) / 100;
            int thresh = flattened.data[numZeros + threshIndex];

            if (m_debug)
                cout << "Threshold set to " << thresh << endl;

            threshold(_extractedImg, _refinedImg, thresh - 1, 255, THRESH_BINARY);

            // remove noise
            morphologyEx(_refinedImg, _refinedImg, MORPH_OPEN, getStructuringElement(MORPH_CROSS, Size(2, 2)));
            noiseRemover(_refinedImg, 10);

            int status = VPtsdector(_extractedImg, _refinedImg, _labeledImg, thresh);

            _refinedImg = _labeledImg.clone();

            if (m_debug) {

                if (status == 0)
                    cout << "Warning::Vanishing Point is NOT detected..." << endl;
                else
                    cout << "Vanishing Point is detected..." << endl;
            }

        } else {
            //ideally it should always be continuous
            throw runtime_error("Matrix is not continuous");
        }

        if (m_debug)
            cout << "Exiting KPercentExtractor::LMsfromImg()" << endl;
    }
}
