#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp>

using namespace cv;
using namespace std;

// assume only checkboard option

class Settings {
    public:
        void write(FileStorage& fs) const;
        void read(const FileNode& node);
        void validate();
	    Size boardSize;
        float squareSize;
        int nrFrames;
        float aspectRatio;
        int delay;
        string input;
        bool showUndistorted;
        bool writePoints;            // Write detected feature points
        bool writeExtrinsics;        // Write extrinsic parameters
        bool writeGrid;              // Write refined 3D target grid points
        bool calibZeroTangentDist;   // Assume zero tangential distortion
        bool calibFixPrincipalPoint; // Fix the principal point at the center
        string outputFileName;       // The name of the file where to write
        bool fixK1;                  // fix K1 distortion coefficient
        bool fixK2;                  // fix K2 distortion coefficient
        bool fixK3;                  // fix K3 distortion coefficient
        bool fixK4;                  // fix K4 distortion coefficient
        bool fixK5;                  // fix K5 distortion coefficient

        vector<string> imageList;
        int flag;
};

void Settings::write(FileStorage& fs) const                        //Write serialization for this class
{
    fs << "{"
                << "BoardSize_Width"  << boardSize.width
                << "BoardSize_Height" << boardSize.height
                << "Square_Size"      << squareSize
                << "Calibrate_NrOfFrameToUse" << nrFrames
                << "Calibrate_FixAspectRatio" << aspectRatio
                << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint
                << "Write_DetectedFeaturePoints" << writePoints
                << "Write_extrinsicParameters"   << writeExtrinsics
                << "Write_gridPoints" << writeGrid
                << "Write_outputFileName"  << outputFileName
                << "Show_UndistortedImage" << showUndistorted
                << "Input_Delay" << delay
                << "Input" << input
        << "}";
}

void Settings::read(const FileNode& node){
    node["BoardSize_Width"] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Square_Size"] >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Write_gridPoints"] >> writeGrid;
    node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Show_UndistortedImage"] >> showUndistorted;
    node["Input_Delay"] >> delay;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
}
void Settings::validate(){
    flag = 0;
    if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= CALIB_FIX_K1;
    if(fixK2)                  flag |= CALIB_FIX_K2;
    if(fixK3)                  flag |= CALIB_FIX_K3;
    if(fixK4)                  flag |= CALIB_FIX_K4;
    if(fixK5)                  flag |= CALIB_FIX_K5;
}

Settings s;
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(
    Settings& s, 
    Size imageSize, 
    Mat&  cameraMatrix, 
    Mat& distCoeffs, 
    vector<vector<Point2f> > imagePoints, 
    float grid_width, 
    bool release_object
);

static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

int main(int argc, char** argv)
{
    const String keys
    = "{help h usage ? |           | print this message            }"
        "{@settings      |default.xml| input setting file            }"
        "{d              |           | actual distance between top-left and top-right corners of "
        "the calibration grid }"
        "{winSize        | 11        | Half of search window for cornerSubPix }";
    CommandLineParser parser(argc, argv, keys);

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = CAPTURING;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    rs2::pipeline pipe;
    auto config = pipe.start();
    rs2::align align_to(RS2_STREAM_COLOR);
    float grid_width = s.squareSize * (s.boardSize.width - 1);

    bool release_object = false;
    if (parser.has("d")) {
        grid_width = parser.get<float>("d");
        release_object = true;
    }

    for (;;){
        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if( mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames ){
            if(runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
                                    release_object))
                mode = CALIBRATED;
            break;
        }
        //! [find_pattern]
        vector<Point2f> pointBuf;

        bool found;
        bool blinkOutput = false;
        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        auto data = pipe.wait_for_frames();
        data = align_to.process(data);

        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();
        auto view = frame_to_mat(color_frame);

        found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);

        if (found) {
            Mat viewGray;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix(
                    viewGray, pointBuf, Size(11,11), Size(-1,-1),
                    TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.001)
                );
            if( mode == CAPTURING &&  // For camera only take new samples after delay time
                (clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) ){
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = true;

                    //save image to disk
                    std::stringstream png_file;
                    std::string stream_name= data.get_profile().stream_name();

                    if( stream_name == "Color" )
                    {
                        png_file << "rs-daemon-output-" << stream_name << ".png";
                        stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(), vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());

                        std::cout << "Saved " << png_file.str()  << ", frame width: " << vf.get_width() << ", height: " << vf.get_height() << std::endl;
                    }
            }
            drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }
        //! [pattern_found]
        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
        string msg = (mode == CAPTURING) ? "100/100" :
                    mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorted)
                msg = cv::format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = cv::format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if(blinkOutput)
            bitwise_not(view, view);
        //! [output_text]
        //------------------------- Video capture  output  undistorted ------------------------------
        //! [output_undistorted]
        if( mode == CALIBRATED && s.showUndistorted )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        imshow("Image View", view);
        char key = (char)waitKey(s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
            s.showUndistorted = !s.showUndistorted;

        if( key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if( s.showUndistorted && !cameraMatrix.empty() )
    {
        Mat view, rview, map1, map2;

        initUndistortRectifyMap(
            cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
            CV_16SC2, map1, map2);

        for(size_t i = 0; i < s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], IMREAD_COLOR);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                        const vector<vector<Point2f> >& imagePoints,
                                        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                        const Mat& cameraMatrix , const Mat& distCoeffs,
                                        vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners){
    corners.clear();

    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        }
    }
}

//! [board_corners]
static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr, vector<Point3f>& newObjPoints,
                            float grid_width, bool release_object)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F); //eye for identity matrix
    if(s.flag & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    //! [fixed_aspect]
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0]);

    // Board imperfectness correction introduced in PR #12772
    // The correction does not make sense for asymmetric and assymetric circles grids
    objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;

    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    int iFixedPoint = -1;
    if (release_object)
        iFixedPoint = s.boardSize.width - 1;
    rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                            cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                            s.flag | CALIB_USE_LU);

    if (release_object) {
        cout << "New board corners: " << endl;
        cout << newObjPoints[0] << endl;
        cout << newObjPoints[s.boardSize.width - 1] << endl;
        cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << endl;
        cout << newObjPoints.back() << endl;
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs);

    return ok;
}

static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                            const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                            double totalAvgErr, const vector<Point3f>& newObjPoints )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if(s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        flagsStringStream << "flags:"
            << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
            << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
            << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
            << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
            << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
            << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
            << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
            << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
            << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if(needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if(needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    /* The code snippet provided is written in C++ and it appears to be creating a matrix named
    `imagePtMat` using the OpenCV library. The matrix is being initialized with the dimensions
    `(int)imagePoints.size()` and `(int)imagePoints[0].size()` with data type `CV_32FC2`, which
    indicates a 32-bit floating point matrix with 2 channels. This matrix is likely being used
    to store image points for further processing or manipulation in an OpenCV application. */
    }

    if( s.writeGrid && !newObjPoints.empty() )
    {
        fs << "grid_points" << newObjPoints;
    }
}

bool runCalibrationAndSave(
    Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints, float grid_width, bool release_object
){
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                            totalAvgErr, newObjPoints, grid_width, release_object);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                        totalAvgErr, newObjPoints);
    return ok;
}