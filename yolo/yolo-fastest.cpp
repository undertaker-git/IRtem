#define ALONE 0
#include "benchmark.h"
#include "cpu.h"
#include "datareader.h"
#include "net.h"
#include "gpu.h"
#include <vector>
#include <math.h>
#include "yolo/yolo-fastest.h"
#include <stdio.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <omp.h>
#include <iostream>
#include <sys/time.h>
#include <QDebug>



using namespace std;
using namespace cv;

float limitedDistance=800.0;
int TrackFrameCount=0;
std::vector<Ptr<Tracker>> trackers;

int detection_interval = 1;
float tpe_add = 2.6;
extern float Surround_temp;
extern float TemperatureOffsetADJ;
vector<Target> globalTargets;
int MtcnnFrameCount=0;


static float getElapse(struct timeval *tv1,struct timeval *tv2)
{
    float t = 0.0f;
    if (tv1->tv_sec == tv2->tv_sec)
        t = (tv2->tv_usec - tv1->tv_usec)/1000.0f;
    else
        t = ((tv2->tv_sec - tv1->tv_sec) * 1000 * 1000 + tv2->tv_usec - tv1->tv_usec)/1000.0f;
    return t;
}

float overlap(float x1, float w1, float x2, float w2)
{
    float l1 = x1 - w1/2;
    float l2 = x2 - w2/2;
    float left = l1 > l2 ? l1 : l2;
    float r1 = x1 + w1/2;
    float r2 = x2 + w2/2;
    float right = r1 < r2 ? r1 : r2;
    return right - left;
}

float box_intersection(box a, box b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    if(w < 0 || h < 0) return 0;
    float area = w*h;
    return area;
}

float box_union(box a, box b)
{
    float i = box_intersection(a, b);
    float u = a.w*a.h + b.w*b.h - i;
    return u;
}

float box_iou(box a, box b)
{
    //return box_intersection(a, b)/box_union(a, b);

    float I = box_intersection(a, b);
    float U = box_union(a, b);
    if (I == 0 || U == 0) {
        return 0;
    }
    return I / U;
}

void do_nms_sort(box* detectedBoxes, int total, float thresh)
{
    int i, j, k;

    //qsort(dets, total, sizeof(detection), nms_comparator_v3);

    for (i = 0; i < total; ++i) {
        if (detectedBoxes[i].ignored == true) continue;
        for(j=i+1;j<total;++j)
        {
            if (box_iou(detectedBoxes[i], detectedBoxes[j]) > thresh) {
                detectedBoxes[j].ignored = true;
            }
        }
    }
}


//摄像头测试
void yoloFastest(std::vector<Target>& Targets, cv::Mat& frame, Mat& frameDisplay,ncnn::Allocator*upa, ncnn::Allocator*pa)
{
    //定义yolo-fastest VOC检测器
    ncnn::Net detector;
    int detector_size_width;
    int detector_size_height;

    detector.load_param("/home/pi/lixicai/Lepton_image_V8/RaspberrypiVideo_Lepton/models/优化后模型/yolo-IRv3-bf16s.param");
    detector.load_model("/home/pi/lixicai/Lepton_image_V8/RaspberrypiVideo_Lepton/models/优化后模型/yolo-IRv3-bf16s.bin");
    detector_size_width  = 160;
    detector_size_height = 120;

    double start = ncnn::get_current_time();
    ncnn::Extractor ex = detector.create_extractor();
    ex.set_blob_allocator(upa);
    ex.set_workspace_allocator(pa);
    ex.set_num_threads(1); //8
    yoloFastestFaceDetect(Targets, frame, frameDisplay, detector, detector_size_width, detector_size_height,ex);

    //std::vector<Target> targets = demo_nms(frame, detector, detector_size_width, detector_size_height,ex);
    double end = ncnn::get_current_time();
    double time = end - start;
}

void yoloFastestFaceDetect(std::vector<Target>& Targets, cv::Mat& image, Mat& frameDisplay, ncnn::Net &detector, int detector_size_width, int detector_size_height,ncnn::Extractor ex)
{
    //imshow("cvImage_160",image);
    //imshow("cvImage_320",frameDisplay);
    float  temperature=35.6;
    static float  temperatureprior=35.6;

    //Surround_temp = Sht20_Test()-10;

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 1, 2, 8);

    IplImage tmp = IplImage(frameDisplay);
    CvArr* arr = (CvArr*)&tmp;

    if (TrackFrameCount % detection_interval == 0)
    {
        //std::cout << "  " << endl;
        Targets.clear();

        TrackFrameCount =0;

        static const char* class_names[] = {"background","Face",
                                            "aeroplane", "bicycle", "bird", "boat",
                                            "bottle", "bus", "car", "cat", "chair",
                                            "cow", "diningtable", "dog", "horse",
                                            "motorbike", "person", "pottedplant",
                                            "sheep", "sofa", "train", "tvmonitor"
                                           };
        cv::Mat bgr = image.clone();
        int img_w = bgr.cols;
        int img_h = bgr.rows;

        ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows, detector_size_width, detector_size_height);

        //数据预处理
        const float mean_vals[3] = {0.f, 0.f, 0.f};
        const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
        in.substract_mean_normalize(mean_vals, norm_vals);

        ex.input("data", in);
        ncnn::Mat out;
        ex.extract("output", out);

        //reset all trackers
        globalTargets.clear();
        if(out.h>0)
        {
            for (int i = 0; i < out.h; i++)
            {
                //printf("num of faces:%d\n",out.h);
                int label;
                float x1, y1, x2, y2, score;
                float pw,ph,cx,cy;
                const float* values = out.row(i);

                //中心位置限制
                static float x1_pre,x2_pre,y1_pre,y2_pre;

                score = values[1];
                label = values[0];

                if(score>=0.85)
                 {
                    x1 = values[2] * img_w;
                    y1 = values[3] * img_h;
                    x2 = values[4] * img_w;
                    y2 = values[5] * img_h;


                    //处理坐标越界问题
                    if(x1<0) x1=0;
                    if(y1<0) y1=0;
                    if(x2<0) x2=0;
                    if(y2<0) y2=0;

                    if(x1>img_w) x1=img_w;
                    if(y1>img_h) y1=img_h;
                    if(x2>img_w) x2=img_w;
                    if(y2>img_h) y2=img_h;


                    //多人的情况
                    if(out.h>=2)
                    {
                        if (i >= 1)
                        {
                            int dx = min(x2, x2_pre) - max(x1, x1_pre);
                            int dy = min(y2, y2_pre) - max(y1, y1_pre);
                            if (dx > 0 && dy > 0)
                            {
                                int area_and =dx*dy;
                                int area_or = (max(x2, x2_pre)-min(x1,x1_pre))*(max(y2, y2_pre)-min(y1,y1_pre));
                                //qDebug()  << "area_and/(area_or*1.0) = " << area_and/(area_or*1.0);
                                if(area_and/(area_or*1.0)>=0.3)
                                {
                                    x1 = max(x1, x1_pre);
                                    y1 = max(y1, y1_pre);
                                    x2 = min(x2, x2_pre);
                                    y2 = min(y2, y2_pre);
                                   // trackers.clear();
                                }
                            }
                        }

                        #if ALONE
                        char text[256];
                        sprintf(text, "%s %.1f%%", class_names[label], score * 100);
                        int baseLine = 0;
                        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                        cv::putText(frameDisplay, text, cv::Point(2*x1, 2*y1),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));
                        #endif

                        cv::Rect rect;
                        rect.x = x1;
                        rect.y = y1;
                        rect.width = (x2-x1);
                        rect.height = (y2-y1);
                        Target curTarget;
                        curTarget.SetTargetRect(rect);
                        curTarget.initTrack(image, Rect2d(rect.x, rect.y, rect.width, rect.height));
                        Targets.push_back(curTarget);

                        x1_pre=x1;
                        x2_pre=x2;
                        y1_pre=y1;
                        y2_pre=y2;

                        int    Area= rect.area();
                        float  Distance= 14227.0*pow(rect.area(),-0.593);
                        cv::rectangle (frameDisplay, cv::Point(x1*2, y1*2), cv::Point(x2*2, y2*2), cv::Scalar(255, 255, 0), 1, 1, 0);
                        if(Distance<=limitedDistance)
                        {
                           // temperature = getROITemperature_quickSort(rect.x/2, rect.y/2, rect.width/2, rect.height/2, 2); //舍去半张脸?
                            //temperature = RealTempByFormula(Surround_temp, temperature,Distance)+tpe_add+TemperatureOffsetADJ;
                            curTarget.SetTargetTemperature(temperature);
                            globalTargets.push_back(curTarget);

                            string AREA = format("%.2f", Distance);
                            string TEMPERATURE = format("%.2f", temperature);

                            //String text_display="("+ AREA +" , " + TEMPERATURE+ ")";
                            String text_display=TEMPERATURE;
                            char * display_text = new char[strlen(text_display.c_str())+1];
                            strcpy(display_text, text_display.c_str());
                            if(temperature>37.5)
                            {
                                cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(255,0,0));
                                //TurnOnLED(1);
                            }
                            else
                            {
                                cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(0,190,168));  //R G B
                                //TurnOffLED(1);
                                //TurnOffLED(2);
                                //TurnOffLED(3);
                            }
                            //imshow("cvImage_160",frameDisplay);
                        }
                    }
                    else //单人的情况
                    {
                        #if ALONE
                        char text[256];
                        sprintf(text, "%s %.1f%%", class_names[label], score * 100);
                        int baseLine = 0;
                        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                        cv::putText(frameDisplay, text, cv::Point(x1*2, y1*2),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));
                        #endif
                        cv::rectangle (frameDisplay, cv::Point(x1*2, y1*2), cv::Point(x2*2, y2*2), cv::Scalar(255, 255, 0), 1, 1, 0);
                        cv::Rect rect;
                        rect.x = x1;
                        rect.y = y1;
                        rect.width = (x2-x1);
                        rect.height = (y2-y1);

                        Target curTarget;
                        curTarget.SetTargetRect(rect);
                        curTarget.initTrack(image, Rect2d(rect.x, rect.y, rect.width, rect.height));
                        Targets.push_back(curTarget);

                        int    Area= rect.area();
                        float  Distance= 14227.0*pow(rect.area(),-0.593);
                        if(Distance<=limitedDistance)
                        {
                            //temperature = getROITemperature_quickSort(rect.x/2, rect.y/2, rect.width/2, rect.height/2, 2); //舍去半张脸?
                            //temperature = RealTempByFormula(Surround_temp, temperature,Distance)+tpe_add+TemperatureOffsetADJ;

                            //temperature= KalmanFilter_lepton(temperature,0.5,5.0);
                            curTarget.SetTargetTemperature(temperature);
                            globalTargets.push_back(curTarget);

                            static bool firstFrameInit=false;
                            if(!firstFrameInit)
                            {
                                 temperatureprior=temperature;
                                 firstFrameInit=true;
                            }
                            if((abs(temperature-temperatureprior)<=1.9) &&(firstFrameInit==true))
                            {
                                temperatureprior=temperature;
                            }

                            //std::cout << " Temperature_kf = " << temperature << std::endl;
                            //std::cout << " Distance = " << Distance << std::endl;
                            string AREA = format("%.2f", Distance);
                            string TEMPERATURE = format("%.2f", temperatureprior);
                            //String text_display="("+ AREA +" , " + TEMPERATURE+ ")";
                            String text_display=TEMPERATURE;
                            char * display_text = new char[strlen(text_display.c_str())+1];
                            strcpy(display_text, text_display.c_str());
                            if(temperatureprior>37.5)
                            {
                                cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(255,0,0));
                                //TurnOnLED(2);
                            }

                            else
                            {
                                cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(0,190,168)); //R G B
                                //TurnOffLED(1);
                                //TurnOffLED(2);
                                //TurnOffLED(3);
                            }
                            //imshow("cvImage_160",frameDisplay);
                        }
                    }
                }
             }
        }
        else
        {
            //TurnOffLED(1);
            //TurnOffLED(2);
            //TurnOffLED(3);
            //temperatureprior=35.6;
        }
    }
   else
   {
        // just track
        for (Target& target : Targets)
        {
            struct timeval  tv1,tv2;
            struct timezone tz1,tz2;

            Ptr<Tracker> curTracker = target.tracker;
            Rect2d bbox(0, 0, 0, 0);

            gettimeofday(&tv1,&tz1);
            if (curTracker->update(image, bbox))
            {
                //rectangle(image, bbox.tl(), bbox.br(), Scalar(255,0,255), 2,8,0);
                cv::Rect rect;
                rect.x = bbox.tl().x;
                rect.y = bbox.tl().y;
                rect.width = bbox.br().x-rect.x;
                rect.height = bbox.br().y-rect.y;

                rect.x = max(0, min(rect.x, 160 - 1));
                rect.y = max(0, min(rect.y, 120 - 1));
                rect.width = rect.x + rect.width >= 160 ? 160 - rect.x - 1 : rect.width;
                rect.height = rect.y + rect.height >= 120 ? 120 - rect.y - 1 : rect.height;

                target.SetTargetRect(rect);

                cv::rectangle (frameDisplay, cv::Point(rect.x*2,rect.y*2), cv::Point(rect.x*2+rect.width*2,rect.y*2+rect.height*2), cv::Scalar(255, 0, 255), 2, 8, 0);

                int    Area= rect.area();
                float  Distance= 14227.0*pow(rect.area(),-0.593);
                //temperature = getROITemperature_quickSort(rect.x/2, rect.y/2, rect.width/2, rect.height/2, 2);
                //temperature = RealTempByFormula(Surround_temp, temperature,Distance)+tpe_add+TemperatureOffsetADJ;

                string AREA = format("%.2f", Distance);
                string TEMPERATURE = format("%.2f", temperature);
                //String text_display="("+ AREA +" , " + TEMPERATURE+ ")";
                String text_display=TEMPERATURE;
                char * display_text = new char[strlen(text_display.c_str())+1];
                strcpy(display_text, text_display.c_str());
                if(temperature>37.5)
                {
                    cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(255,0,0));
                    //TurnOnLED(3);
                }

                else
                {
                    cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(0,190,168));  //R G B
                    //TurnOffLED(1);
                    //TurnOffLED(2);
                    //TurnOffLED(3);
                }


                gettimeofday(&tv2,&tz2);
                //printf( "%s = %g ms \n ", "Track All time", getElapse(&tv1, &tv2) );
            }
        }
    }
    TrackFrameCount++;
}


// PlotRectangle_RGB
void PlotRectangle_RGB(Mat imageRgb)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 1, 2, 8);

    IplImage tmp = IplImage(imageRgb);
    CvArr* arr = (CvArr*)&tmp;

    int offset_x=5;
    int offset_y=2;
    int offset_h=2;
    int offset_w=2;
    Mat cvRioimg;
    Mat MtcnnRoiImage;
    Rect2d ROI_box;

    for (Target& target : globalTargets)
    {
        cv::Rect rect = target.GetTargetRect();
        //cv::rectangle (imageRgb, cv::Point(rect.x*4,rect.y*4), cv::Point(rect.x*4+rect.width*4,rect.y*4+rect.height*4), cv::Scalar(255, 255, 0), 1.5, 8, 0);

        ROI_box.x = (rect.x+offset_x)*4;
        ROI_box.y = (rect.y+offset_y)*4;
        ROI_box.width  = (rect.width-offset_w)*4;
        ROI_box.height =  (rect.height-offset_h)*4;

        cv::rectangle (imageRgb, cv::Point(ROI_box.x,ROI_box.y), cv::Point(ROI_box.x+ROI_box.width,ROI_box.y+ROI_box.height), cv::Scalar(0, 0, 255), 2, 8, 0);

        //imageRgb.copyTo(cvRioimg);
        //Rect MtcnnRect(ROI_box.x, ROI_box.y, ROI_box.width, ROI_box.height);
        //cvRioimg(MtcnnRect).copyTo(MtcnnRoiImage);
        //MtcnnDetection(MtcnnRoiImage,imageRgb,ROI_box.x,ROI_box.y);
        //FaceDetect(MtcnnRoiImage,imageRgb,ROI_box.x,ROI_box.y,ROI_box);
        //MtcnnDetection(imageRgb,imageRgb);

        float temperature = target.GetTargetTemperature();
        string TEMPERATURE = format("%.2f", temperature);
        String text_display=TEMPERATURE;
        char * display_text = new char[strlen(text_display.c_str())+1];
        strcpy(display_text, text_display.c_str());
        //cvPutText(arr,display_text, Point2d(rect.x*4,4*(rect.y-3)),&font, CV_RGB(0,190,168)); //R G B
        cvPutText(arr,display_text, Point2d((rect.x+offset_x)*4,4*(rect.y-3)),&font, CV_RGB(255,0,0)); //R G B
    }
    MtcnnFrameCount++;
}


// FaceDetect_RGB
void FaceDetect_RGB_Roi(Mat imageRgb)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 1, 2, 8);

    IplImage tmp = IplImage(imageRgb);
    CvArr* arr = (CvArr*)&tmp;

    int offset_x=5;
    int offset_y=2;
    int offset_h=2;
    int offset_w=2;
    Mat cvRioimg;
    Mat MtcnnRoiImage;
    Rect2d ROI_box;
    //printf( "globalTargets %d \r\n ", globalTargets.size());

    for (Target& target : globalTargets)
    {
        if(globalTargets.size()!=0)
        {
            cv::Rect rect = target.GetTargetRect();
            //cv::rectangle (imageRgb, cv::Point(rect.x*4,rect.y*4), cv::Point(rect.x*4+rect.width*4,rect.y*4+rect.height*4), cv::Scalar(255, 255, 0), 1.5, 8, 0);

            ROI_box.x = (rect.x+offset_x)*4;
            ROI_box.y = (rect.y+offset_y)*4;
            ROI_box.width  = (rect.width-offset_w)*4;
            ROI_box.height =  (rect.height-offset_h)*4;

            cv::rectangle (imageRgb, cv::Point(ROI_box.x,ROI_box.y), cv::Point(ROI_box.x+ROI_box.width,ROI_box.y+ROI_box.height), cv::Scalar(0, 0, 255), 1.5, 8, 0);

            imageRgb.copyTo(cvRioimg);
            Rect MtcnnRect(ROI_box.x, ROI_box.y, ROI_box.width, ROI_box.height);
            cvRioimg(MtcnnRect).copyTo(MtcnnRoiImage);
            //MtcnnDetection(MtcnnRoiImage,imageRgb,ROI_box.x,ROI_box.y);
            //FaceDetect(MtcnnRoiImage,imageRgb,ROI_box.x,ROI_box.y,ROI_box);
            //MtcnnFaceDetect_ROI(MtcnnRoiImage,imageRgb,ROI_box.x,ROI_box.y,ROI_box,globalTargets.size());
            //MtcnnDetection(imageRgb,imageRgb);

            float temperature = target.GetTargetTemperature();
            string TEMPERATURE = format("%.2f", temperature);
            String text_display=TEMPERATURE;
            char * display_text = new char[strlen(text_display.c_str())+1];
            strcpy(display_text, text_display.c_str());
            //cvPutText(arr,display_text, Point2d(rect.x*4,4*(rect.y-3)),&font, CV_RGB(0,190,168)); //R G B
            cvPutText(arr,display_text, Point2d((rect.x+offset_x)*4,4*(rect.y-3)),&font, CV_RGB(255,0,0)); //R G B
        }
    }
    MtcnnFrameCount++;
}



// FaceDetect_RGB
void FaceDetect_RGB(Mat imageRgb,bool detectionFlag)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 1, 2, 8);

    IplImage tmp = IplImage(imageRgb);
    CvArr* arr = (CvArr*)&tmp;

    int offset_x=5;
    int offset_y=2;
    int offset_h=2;
    int offset_w=2;
    Mat MtcnnRoiImage;
    Rect2d ROI_box;

    imageRgb.copyTo(MtcnnRoiImage);
    //MtcnnFaceDetect(MtcnnRoiImage,imageRgb);

    if(detectionFlag==true)
    {
        for (Target& target : globalTargets)
        {
            if(globalTargets.size()!=0)
            {
                cv::Rect rect = target.GetTargetRect();
                ROI_box.x = (rect.x+offset_x)*4;
                ROI_box.y = (rect.y+offset_y)*4;
                ROI_box.width  = (rect.width-offset_w)*4;
                ROI_box.height =  (rect.height-offset_h)*4;

                //cv::rectangle (imageRgb, cv::Point(ROI_box.x,ROI_box.y), cv::Point(ROI_box.x+ROI_box.width,ROI_box.y+ROI_box.height), cv::Scalar(0, 0, 255), 1.5, 8, 0);
                float temperature = target.GetTargetTemperature();
                string TEMPERATURE = format("%.2f", temperature);
                String text_display=TEMPERATURE;
                char * display_text = new char[strlen(text_display.c_str())+1];
                strcpy(display_text, text_display.c_str());
                //cvPutText(arr,display_text, Point2d((rect.x+offset_x)*4,4*(rect.y-3)),&font, CV_RGB(255,0,0)); //R G B
                //cvPutText(arr,display_text, Point2d((rect.x+offset_x+3)*4,4*(rect.y+5)),&font, CV_RGB(255,0,0)); //R G B
            }
        }
        MtcnnFrameCount++;
    }
}




