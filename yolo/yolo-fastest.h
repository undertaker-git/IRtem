#ifndef YOLOFASTEST_H
#define YOLOFASTEST_H

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv.hpp>

#include "net.h"
#include "target.h"
#include <mtcnn.h>


typedef struct box {
    float x, y, w, h;
    bool ignored;
} box;

void yoloFastest(std::vector<Target>& Targets, cv::Mat& frame, cv::Mat& frameDisplay,ncnn::Allocator*upa, ncnn::Allocator*pa);
void yoloFastestFaceDetect(std::vector<Target>& Targets, cv::Mat& image, Mat& frameDisplay, ncnn::Net &detector, int detector_size_width, int detector_size_height,ncnn::Extractor ex);

void  do_nms_sort(box* detectedBoxes, int total, float thresh);
float box_iou(box a, box b);
float box_union(box a, box b);
float overlap(float x1, float w1, float x2, float w2);
void  PlotRectangle_RGB(Mat imageRgb);
void  FaceDetect_RGB(Mat imageRgb,bool detectionFlag);
void  FaceDetect_RGB_Roi(Mat imageRgb);

#endif // YOLOFASTEST_H
