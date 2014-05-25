#include <vector>
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
//using namespace cv;

IplImage* GetThresholdedImage(IplImage* img)
{
  // Convert the image into an HSV image
  IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
  cvCvtColor(img, imgHSV, CV_BGR2HSV);

  IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

  // Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
  //cvInRangeS(imgHSV, cvScalar(112, 100, 100), cvScalar(124, 255, 255), imgThreshed);
  cvInRangeS(imgHSV, cvScalar(50, 80, 50), cvScalar(80, 255, 255), imgThreshed);

  cvReleaseImage(&imgHSV);

  return imgThreshed;
}


int main(int argc, char *argv[])
{

  // Window setup
  cvNamedWindow("video");
  cvNamedWindow("videoBlur");
  cvNamedWindow("green");
  cvNamedWindow("greenFilled");
  cvNamedWindow("framePoints");

  vector<cv::KeyPoint> keyPoints;

  // This image holds the "scribble" data...
  // the tracked positions of the ball
  //IplImage* imgScribble = NULL;

  // Video capture
  cv::VideoCapture cap;
  if(argc > 1)
    {
      cap.open(string(argv[1]));
    }
  else
    {
      //cap.open(CV_CAP_ANY);
      cap.open(0);
    }
  if(!cap.isOpened())
    {
      printf("Error: could not load a camera or video.\n");
    }
  cv::Mat frame;
  //cv::namedWindow("video", 1);
  for(;;)
  {
    cv::waitKey(20);
    cap >> frame;
    if(!frame.data)
      {
	printf("Error: no frame data.\n");
	break;
      }

    // If this is the first frame, we need to initialize it
    /*
    if(imgScribble == NULL)
      {
	imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
      }
    */

    // Do image processing
    cv::Mat frameBlur;
    blur(frame, frameBlur, cv::Size( 5, 5 ), cv::Point(-1,-1));
    
    IplImage copy = frameBlur;
    IplImage* newCopy = &copy;
    IplImage* green = GetThresholdedImage(newCopy);
    cv::Mat greenMat(green);

    cv::Mat greenDilate;
    int dilation_size = 5;
    cv::Mat dilater = getStructuringElement(cv::MORPH_RECT,
					    cv::Size(2*dilation_size + 1, 2*dilation_size+1),
					    cv::Point(dilation_size, dilation_size));
    dilate(greenMat, greenDilate, dilater);      /// Apply the dilation operation

    cv::Mat greenErode;
    int erosion_size = 5;
    cv::Mat eroder = getStructuringElement(cv::MORPH_RECT,
					    cv::Size(2*erosion_size + 1, 2*erosion_size+1),
					    cv::Point(erosion_size, erosion_size));
    erode(greenMat, greenErode, eroder);      /// Apply the dilation operation

    // Implement SimpleBlobDetector
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 0;
    params.maxThreshold = 256;
    //params.thresholdStep = 5;
    
    params.minArea = 1000; 
    params.maxArea = 8000000;

    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByInertia = false;
    params.filterByConvexity = false;

    //line( src, Point(0, src.rows-1), Point( src.cols-1, src.rows-1 ), Scalar::all(255) );

    cv::SimpleBlobDetector blobDetector( params );
    blobDetector.create("SimpleBlob");
    blobDetector.detect(greenErode, keyPoints);

    cv::Mat framePoints;
    drawKeypoints(frame, keyPoints, framePoints, CV_RGB(0,255,0), cv::DrawMatchesFlags::DEFAULT);

    for(vector<cv::KeyPoint>::const_iterator i = keyPoints.begin(); i != keyPoints.end(); ++i)
      cout << (*i).pt << endl;

    // Display stuff
    cv::imshow("video", frame);
    cv::imshow("videoBlur", frameBlur);
    cv::imshow("green", greenMat);
    cv::imshow("greenFilled", greenErode);
    cv::imshow("framePoints", framePoints);

    //cvReleaseImage(&newCopy);
    //cvReleaseImage(&green);
  }
}
