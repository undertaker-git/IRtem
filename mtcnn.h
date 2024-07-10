#ifndef MTCNN_H
#define MTCNN_H

#include <iostream>
#include <omp.h>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <math.h>
#include <iostream>
#include <sys/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking.hpp>
#include <cstring>
#include "ncnn-master/src/net.h"

#define KALMAN_BEGIN 100
#define KALMAN_INTERVAL 10

using namespace std;
using namespace cv;

struct Bbox
{
    float score;
    int x1;
    int y1;
    int x2;
    int y2;
    float area;
    bool exist;
    float ppoint[10];
    float regreCoord[4];
};

struct orderScore
{
    float score;
    int oriOrder;
};

void draw_rectangle(int event, int x, int y, int flags, void*);
bool cmpScore(orderScore lsh, orderScore rsh);
static float getElapse(struct timeval *tv1,struct timeval *tv2);
void generateBbox(ncnn::Mat score, ncnn::Mat location, std::vector<Bbox>& boundingBox_, std::vector<orderScore>& bboxScore_, float scale);
void nms(std::vector<Bbox> &boundingBox_, std::vector<orderScore> &bboxScore_, const float overlap_threshold, string modelname);
void refineAndSquareBbox(vector<Bbox> &vecBbox, const int &height, const int &width);
void detect(ncnn::Mat& img_, std::vector<Bbox>& finalBbox_);
void MtcnnDetection(Mat ImgDetect,Mat ImgShow,int offset_x,int offset_y);
void FaceDetect(Mat ImgDetect,Mat ImgShow,int offset_x,int offset_y,Rect2d roibox);
void MtcnnFaceDetect_ROI(Mat ImgDetect,Mat ImgShow,int offset_x,int offset_y,Rect2d roibox,int RoiBoxNum);
void MtcnnFaceDetect(Mat ImgDetect,Mat ImgShow);

#endif // MTCNN_H
