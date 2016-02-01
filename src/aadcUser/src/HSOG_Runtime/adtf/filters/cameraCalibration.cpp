#include "cameraCalibration.h"

ADTF_FILTER_PLUGIN(FILTER_CLASS_LABEL, FILTER_CLASS_ID, cameraCalibration)

cameraCalibration::cameraCalibration(const tChar* __info):cFilter(__info)
{
}

cameraCalibration::~cameraCalibration()
{
}

tResult cameraCalibration::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{
		// create the input VideoPin
		RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_IN",IPin::PD_Input, static_cast<IPinEventSink*>(this))); 
		RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

		// create the output VideoPin
		RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_OUT", adtf::IPin::PD_Output, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));

	}
	else if (eStage == StageGraphReady)
	{
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		UpdateImageFormat(pTypeVideo->GetFormat());
	}
	RETURN_NOERROR;
}

tResult cameraCalibration::Shutdown(tInitStage eStage, __exception)
{		
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cameraCalibration::OnPinEvent(IPin* pSource,
		tInt nEventCode,
		tInt nParam1,
		tInt nParam2,
		IMediaSample* pMediaSample)
{
	if(pSource != &m_oPinInputVideo)
		RETURN_NOERROR;

	switch (nEventCode)
	{
		case IPinEventSink::PE_MediaSampleReceived:
			{
				ProcessVideo(pMediaSample);		
				break;
			}
		case IPinEventSink::PE_MediaTypeChanged:
			{
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

				// update video format
				UpdateImageFormat(pTypeVideo->GetFormat());
				break;
			}
	}

	RETURN_NOERROR;
}

tResult cameraCalibration::ProcessVideo(adtf::IMediaSample* pISample)
{
	RETURN_IF_POINTER_NULL(pISample);

	// creating new temporary variables
	__adtf_sample_read_lock(pISample, void, l_pSrcBuffer);

	int rows = m_sInputFormat.nHeight;
	int cols = m_sInputFormat.nWidth;

	Mat image(rows, cols, CV_8UC3, const_cast<void*>(l_pSrcBuffer));
	Mat copy = image.clone();

	Scalar red(0, 0, 255);
	// horizontal line
	Point pt1(0, rows/2.0);
	Point pt2(cols, rows/2.0);
	line(copy,pt1, pt2, red);

	// vertical line
	Point pt3(cols/2.0, 0);
	Point pt4(cols/2.0, rows);
	line(copy, pt3, pt4, red);

	cObjectPtr<IMediaSample> pNewRGBSample;
	if (IS_OK(AllocMediaSample(&pNewRGBSample)))
	{							
		tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
		pNewRGBSample->Update(tmStreamTime, copy.data, copy.dataend - copy.datastart, 0);
		m_oPinOutputVideo.Transmit(pNewRGBSample);
	}	

	RETURN_NOERROR;
}

tResult cameraCalibration::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult cameraCalibration::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult cameraCalibration::UpdateImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		// update input format
		m_sInputFormat = (*pFormat);

		// set the videoformat of the video output pin
		m_oPinOutputVideo.SetFormat(&m_sInputFormat, NULL);
	}

	RETURN_NOERROR;
}
