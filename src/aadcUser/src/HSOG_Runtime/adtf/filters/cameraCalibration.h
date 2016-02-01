#pragma once

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <opencv2/opencv.hpp>

using namespace adtf;
using namespace cv;

#define FILTER_CLASS_ID "adtf.aadc.CameraCalibration"
#define FILTER_CLASS_LABEL "HSOG CameraCalibration"
#define FILTER_VERSION_LABEL "Beta Version"

class cameraCalibration : public cFilter
{
	ADTF_DECLARE_FILTER_VERSION(FILTER_CLASS_ID, FILTER_CLASS_LABEL, adtf::OBJCAT_Tool, "1.0", 1, 0, 0, FILTER_VERSION_LABEL);	

	public:
	cameraCalibration(const tChar* __info);
	~cameraCalibration();
	tResult Init(tInitStage eStage, __exception);

	public:
	tResult OnPinEvent(adtf::IPin* pSource,
			tInt nEventCode,
			tInt nParam1,
			tInt nParam2,
			adtf::IMediaSample* pMediaSample) override;

	protected:
	cVideoPin m_oPinInputVideo;
	cVideoPin m_oPinOutputVideo;

	tResult Start(__exception);
	tResult Stop(__exception);
	tResult Shutdown(tInitStage eStage, __exception = NULL);

	private:
	tResult ProcessVideo(adtf::IMediaSample* pSample);
	tResult UpdateImageFormat(const tBitmapFormat* pFormat);

	tBitmapFormat m_sInputFormat;
};
