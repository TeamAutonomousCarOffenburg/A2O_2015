#include <cstdio>
#include <fstream>
#include <sstream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#if CV_MAJOR_VERSION >= 3
#include <opencv2/calib3d.hpp>
#endif

#include "utils/geometry/Pose2D.h"
#include "utils/geometry/Pose3D.h"
#include "utils/geometry/Geometry.h"
#include "BitCodeSignDetection.h"

using namespace cv;
using namespace A2O;
using namespace std;

const string BitCodeSignDetection::_markersYML=

"%YAML:1.0\n"
"nmarkers: 12\n"
"markersize: 3\n"
"marker_0: \"010010111\"\n"
"marker_1: \"001100010\"\n"
"marker_2: \"001001111\"\n"
"marker_3: \"110011101\"\n"
"marker_4: \"011001010\"\n"
"marker_5: \"110101111\"\n"
"marker_6: \"000111101\"\n"
"marker_7: \"011101000\"\n"
"marker_8: \"001010101\"\n"
"marker_9: \"010011001\"\n"
"marker_10: \"110011010\"\n"
"marker_11: \"010010100\"\n"
;

void BitCodeSignDetection::init(){

	// TODO Cleanup: aruco needs a file to load a dictionary from
	string tmp(tmpnam(NULL));

	stringstream fileName;
	fileName << tmp;
	fileName << ".yml";
	fileName >> tmp;

	ofstream tmpFile(tmp);
	tmpFile << _markersYML;
	tmpFile.flush();
	_Dictionary.fromFile(tmp);


	if(_Dictionary.size()==0)
	{
		cerr << "Could not find sign dictionary" << endl;
	}


	aruco::HighlyReliableMarkers::loadDictionary(_Dictionary);

	_detector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
	_detector.setThresholdParams( 21, 7);
	_detector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
	_detector.setWarpSize((_Dictionary[0].n()+2)*8);
	_detector.setMinMaxSize(0.005, 0.5);

	_cameraMatrix = (Mat_<double>(3,3) << 526.37013657, 0.00000000, 320.0, 0.00000000, 526.37013657, 240.0, 0.00000000, 0.00000000, 1.00000000);

}

void BitCodeSignDetection::setFrame(cv::Mat& frame)
{
	_updateFrame.lock();
	_frame = frame;
	_updateFrame.unlock();
}

cv::Mat BitCodeSignDetection::getFrame()
{
	_updateFrame.lock();
	cv::Mat ret(_frame);
	_updateFrame.unlock();
	return ret;
}

void BitCodeSignDetection::start()
{
	Runnable::start("BitCodeSignDetection");
}

void BitCodeSignDetection::stop()
{
	Runnable::stop();
}

void BitCodeSignDetection::notifyPossibleNewFrame()
{
	Worker::notify();
}


void BitCodeSignDetection::run(){
	while(!m_stop)
	{
		detectSigns();
	}
}

void BitCodeSignDetection::setResult(const SignResult& result)
{
	_resultLock.lock();
	_result = boost::make_shared<SignResult>(result);
	_resultLock.unlock();
}

SignResult::Ptr BitCodeSignDetection::getSignResult()
{
	_resultLock.lock();
	SignResult::Ptr res = _result;
	_result = SignResult::Ptr();
	_resultLock.unlock();
	return res;
}


void printQuatAngles(Eigen::Quaterniond& quat)
{
	double heading = atan2(2*quat.y()*quat.w()-2*quat.x()*quat.z(), 1 -2 * quat.y()*quat.y()-2*quat.z()*quat.z());

	double attitude = asin(2*quat.x()*quat.y() + 2 *quat.z()*quat.w());

	double bank = atan2(2*quat.x()*quat.w()- 2 * quat.y()*quat.z(), 1 - 2 *quat.x() * quat.x() -2 * quat.z() * quat.z());

	cout << "heading: " << Angle(heading).deg() << " attitude: " << Angle(attitude).deg() << " bank: " << Angle(bank).deg() << endl;
}

void BitCodeSignDetection::detectSigns(){
	static void* oldFrameDataLocation = nullptr;

	if(!wait())
		return;

	cv::Mat frame = getFrame();
	if(frame.empty() || frame.data == oldFrameDataLocation)
		return;

	oldFrameDataLocation = frame.data;



	aruco::CameraParameters cameraParameters;
	vector<aruco::Marker> markers;

	_detector.detect(frame, markers, cameraParameters, 0.12); 


	cv::Mat output = frame.clone();

	SignResult result;

	for(auto it = markers.begin(); it != markers.end(); ++it)
	{
		aruco::Marker marker = *it;

		if(_debug)
		{
			marker.draw(output, cv::Scalar(0,0,255));

			namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "Display window", output);                   // Show our image inside it.
			waitKey(1);
		}

		marker.calculateExtrinsics(_signSizeM, _cameraMatrix);

		Mat rotationMat;
		Rodrigues(marker.Rvec, rotationMat);

		// convert to eigen mats
		Eigen::Matrix3d eigenRot;
		cv2eigen(rotationMat, eigenRot); 

		Eigen::Vector3d eigenTrans;
		cv2eigen(marker.Tvec, eigenTrans);

		// TODO rotate correctly, so we can use the resulting quat for the pose
		Eigen::Quaterniond quat(eigenRot);
		Eigen::AngleAxisd axis1(M_PI/2.0, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd axis2((M_PI/2.0), Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd axis3((M_PI/2.0), Eigen::Vector3d::UnitX());

		Eigen::Quaterniond rotatedQuat = Eigen::Quaterniond(axis1) * Eigen::Quaterniond(axis2) * Eigen::Quaterniond(axis3) * quat;

		// this gives us the angle we want
		double bank = atan2(2*rotatedQuat.x()*rotatedQuat.w()- 2 * rotatedQuat.y()*rotatedQuat.z(), 1 - 2 *rotatedQuat.x() * rotatedQuat.x() -2 * rotatedQuat.z() * rotatedQuat.z());

		Pose3D signPose(eigenTrans, quat);
		Pose3D signPose2(signPose.z(), -signPose.x(),-signPose.y()); 

		Pose2D signPose2d(signPose2.x(), signPose2.y(), Angle(bank)); 
		result.addSign(RoadSign(static_cast<SignType>(marker.id), signPose2d));
	}
	if(result.getSigns().size() > 0)
	{
		setResult(result);
	}
}
