#ifndef TARGET_H
#define TARGET_H

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <qmap.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <cstring>

using namespace std;
using namespace cv;


class Target{
private:
    cv::Rect rect;
    float Temperature;
    float temperature_Previous=36.5;
    int locate;
public:
    QList<float> TemperatureArray;

    Ptr<Tracker> tracker;
    void SetTargetRect(cv::Rect rect);
    cv::Rect GetTargetRect();
    void  SetTargetTemperature(float temperature);
    float GetTargetTemperature();
    float GetTargetPreviousTemperature();
    void  UpdatePreviousTemperature();
    float GetMajorityTemperature(float temperature);
    bool initTrack( InputArray image, const Rect2d& boundingBox );
    void addLocate();
    int getLocate();
};

#endif // TARGET_H
