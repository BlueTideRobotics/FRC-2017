#pragma once
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

namespace grip {

/**
* GripPipeline class.
* 
* An OpenCV pipeline generated by GRIP.
*/
class GripPipeline {
	private:
		cv::Mat resizeImageOutput;
		cv::Mat hslThresholdOutput;
		cv::Mat cvErodeOutput;
		void resizeImage(cv::Mat &, double , double , int , cv::Mat &);
		void hslThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void cvErode(cv::Mat &, cv::Mat &, cv::Point &, double , int , cv::Scalar &, cv::Mat &);

	public:
		GripPipeline();
		void Process(cv::Mat& source0);
		cv::Mat* GetResizeImageOutput();
		cv::Mat* GetHslThresholdOutput();
		cv::Mat* GetCvErodeOutput();
};


} // end namespace grip


