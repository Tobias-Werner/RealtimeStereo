#include <vector>
#include <unistd.h>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


class Calibration {

private:
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;
    Size boardSize;
    Size imageSize;

public:

    Calibration(Size boardSize = Size(19, 12), Size imageSize = Size(1024, 768)) {
        this->boardSize = boardSize;
        this->imageSize = imageSize;
    }

    void addChessboardImage(const Mat &image) {

        vector<Point2f> boardCorners;
        Mat grayscaledImage = Mat(cv::Mat::zeros(this->imageSize, CV_32FC1));

        vector<Point3f> obj;
        for (int j = 0; j < this->boardSize.height * this->boardSize.width; j++)
            obj.push_back(Point3f(j / this->boardSize.width, j % this->boardSize.width, 0.0f));


        bool found = false;

        cvtColor(image, grayscaledImage, cv::COLOR_BGR2GRAY, 2);
        //imshow("win1", grayscaledImage);

        found = findChessboardCorners(grayscaledImage, this->boardSize, boardCorners);

        if (found) {

            InputArray arr(boardCorners);
            auto mat = arr.getMat();
            mat.convertTo(mat, CV_32F);

            cornerSubPix(grayscaledImage, mat, Size(11, 11), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            //drawChessboardCorners(grayscaledImage, boardSize, mat, found);

            this->imagePoints.push_back(mat);
            this->objectPoints.push_back(obj);

        }

    }

    void calibrate(Mat &intrinsic, Mat &distCoeffs) {
        intrinsic = Mat(3, 3, CV_32FC1);
        vector<Mat> rvecs;
        vector<Mat> tvecs;

        intrinsic.ptr<float>(0)[0] = 1;
        intrinsic.ptr<float>(1)[1] = 1;

        calibrateCamera(this->objectPoints, this->imagePoints, this->imageSize, intrinsic, distCoeffs, rvecs, tvecs);
    }


};


void showImages() {
    VideoCapture cap1;
    VideoCapture cap2;

    cap1.open(4);
    cap2.open(2);

    size_t counter = 0;

    Calibration calibration;
    Mat intrinsic;
    Mat distCoeffs;

    bool run = false;

    for (;;) {

        Mat frame1;
        Mat frame2;
        cap1 >> frame1;
        cap2 >> frame2;

        if (counter == 0) {
            imshow("Cam 1", frame1);
            //imshow("Cam 2", frame2);
        }

        int key = waitKey(10);


        if (counter % 20 == 0) {
            imshow("Cam 1", frame1);
            calibration.addChessboardImage(frame1);
            //imshow("Cam 2", frame2);
        }


        switch (key) {
            case 27:
                return;

            case 120: {
                calibration.calibrate(intrinsic, distCoeffs);
                run = true;
                break;
            }

        }

        if (run) {
            Mat imageUndistorted;
            undistort(frame1, imageUndistorted, intrinsic, distCoeffs);
            imshow("Cam 1", frame1);
            imshow("Cam 1 - entzerrt", imageUndistorted);
        }

        counter++;
    }
}

void disparity() {
    Mat disparity;

    auto matcher = StereoSGBM::create(0, 16, 5);
    //auto matcher = StereoBM::create(16*4, 17);


    VideoCapture cap1;
    VideoCapture cap2;

    cap1.open(4);
    cap2.open(2);

    size_t counter = 0;


    for (;;) {

        Mat frame1;
        cap1 >> frame1;

        Mat frame2;
        cap2 >> frame2;

/*        frame1.convertTo(frame1, CV_8U);
        frame2.convertTo(frame2, CV_8U);*/

        //cvtColor(frame1, frame1, CV_BGR2GRAY);
        //cvtColor(frame2, frame2, CV_BGR2GRAY);

        matcher->compute(frame1, frame2, disparity);

        imshow("Disparity", disparity);


        int key = waitKey(10);

        switch (key) {
            case 27:
                return;
        }

        counter++;
    }

}


void stereoCalibration() {
    VideoCapture cap1;
    VideoCapture cap2;

    cap1.open(4);
    cap2.open(2);

    size_t chessboardHitCounter = 0;
    size_t frameCounter = 0;

    Mat frame1;
    Mat frame2;

    vector<vector<Point2f> > imagePoints1, imagePoints2;
    vector<vector<Point2f> > left_img_points, right_img_points;
    vector<vector<Point3f>> objectPoints;


    while (chessboardHitCounter < 20) {

        cap1 >> frame1;
        cap2 >> frame2;


        if (frameCounter % 10 == 0) {

            imshow("cam 1 original", frame1);
            imshow("cam 2 original", frame2);
            auto c = waitKey(10);
            if (c == 27)
                return;

            Size boardSize(19, 12);
            Size imageSize(1024, 768);

            vector<Point2f> boardCorners1, boardCorners2;
            Mat grayImage1 = Mat(cv::Mat::zeros(imageSize, CV_32FC1));
            Mat grayImage2 = Mat(cv::Mat::zeros(imageSize, CV_32FC1));

            cvtColor(frame1, grayImage1, cv::COLOR_BGR2GRAY, 2);
            cvtColor(frame2, grayImage2, cv::COLOR_BGR2GRAY, 2);

            bool found1 = findChessboardCorners(grayImage1, boardSize, boardCorners1);
            bool found2 = findChessboardCorners(grayImage2, boardSize, boardCorners2);

            if (!found1) cout << "Chessboard in 1 nicht gefunden" << endl;
            if (!found2) cout << "Chessboard in 2 nicht gefunden" << endl;

            if (found1 && found2) {

                InputArray arr1(boardCorners1);
                auto mat1 = arr1.getMat();
                mat1.convertTo(mat1, CV_32F);
                cornerSubPix(grayImage1, mat1, Size(11, 11), Size(-1, -1),
                             TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

                InputArray arr2(boardCorners2);
                auto mat2 = arr2.getMat();
                mat2.convertTo(mat2, CV_32F);
                cornerSubPix(grayImage2, mat2, Size(11, 11), Size(-1, -1),
                             TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

                vector<Point3f> obj;
                for (int j = 0; j < boardSize.height * boardSize.width; j++)
                    obj.push_back(Point3f(j / boardSize.width, j % boardSize.width, 0.0f));

                imagePoints1.push_back(mat1);
                imagePoints2.push_back(mat2);
                objectPoints.push_back(obj);

                chessboardHitCounter++;
            }
        }

        frameCounter++;
    }


    for (int i = 0; i < imagePoints1.size(); i++) {
        vector<Point2f> v1, v2;
        for (int j = 0; j < imagePoints1[i].size(); j++) {
            v1.push_back(Point2f((double) imagePoints1[i][j].x, (double) imagePoints1[i][j].y));
            v2.push_back(Point2f((double) imagePoints2[i][j].x, (double) imagePoints2[i][j].y));
        }
        left_img_points.push_back(v1);
        right_img_points.push_back(v2);
    }

    Mat K1, K2, R, F, E;
    Vec3d T;
    Mat D1, D2;
    int flag = 0;
    flag |= CV_CALIB_FIX_INTRINSIC;

    cout << "Stereo calibrate" << endl;

    stereoCalibrate(objectPoints, left_img_points, right_img_points, K1, D1, K2, D2, frame1.size(), R, T, E, F);

    cv::Mat R1, R2, P1, P2, Q;
    stereoRectify(K1, D1, K2, D2, frame1.size(), R, T, R1, R2, P1, P2, Q);

    Mat disparity;
    auto matcher = StereoSGBM::create(50, 16 * 17, 9);
    matcher->setP1(11 * 11 * 9);
    matcher->setP1(11 * 11 * 9 * 4);

    while (true) {

        cap1 >> frame1;
        cap2 >> frame2;

        cv::Mat lmapx, lmapy, rmapx, rmapy;
        cv::Mat imgU1, imgU2;

        cv::initUndistortRectifyMap(K1, D1, R1, P1, frame1.size(), CV_32F, lmapx, lmapy);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, frame2.size(), CV_32F, rmapx, rmapy);
        cv::remap(frame1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
        cv::remap(frame2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);

        //imshow("cam 1 original", frame1);
        //imshow("cam 2 original", frame2);

        //imshow("cam 1 undist", imgU1);
        //imshow("cam 2 undist", imgU2);

        cvtColor(imgU1, imgU1, cv::COLOR_BGR2GRAY, 2);
        cvtColor(imgU2, imgU2, cv::COLOR_BGR2GRAY, 2);

        matcher->compute(imgU1, imgU2, disparity);

        Mat threed;

        //reprojectImageTo3D(disparity, threed, Q);

        imshow("disparity", disparity * 4);

        auto c = waitKey(10);
        if (c == 27)
            return;
    }
}

int main(int argc, char **argv) {

    //disparity();
    stereoCalibration();

    //showImages();

/*
    Mat imageUndistorted;
    undistort(image, imageUndistorted, intrinsic, distCoeffs);

    imshow("win1", imageUndistorted);
  */



    return 0;
}

