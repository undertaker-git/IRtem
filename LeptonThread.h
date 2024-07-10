#ifndef TEXTTHREAD
#define TEXTTHREAD

#include <ctime>
#include <stdint.h>


#include <QThread>
#include <QtCore>
#include <QPixmap>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <QDebug>
#include "net.h"
#include "target.h"

using namespace cv;
using namespace std;

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_SEGMENT 60
#define SEGMENT_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_SEGMENT)

#define FRAME_SIZE   (FRAME_SIZE_COL*FRAME_SIZE_ROW)

#define VALID_TTT_PACKET 20

#define FPS 27;

#define MODICOL 1280
#define MODIROW 720
#define ORIGCOL 320
#define ORIGROW 240

class LeptonThread : public QThread
{
  Q_OBJECT

public:
  LeptonThread();
  ~LeptonThread();
  int imagecapture_flag  = 0;
  int video_capture_flag = 0;
  int image_num = 0;
  int mode_flag = 0;
  int maxTemperature_flag = 0;
  int minTemperature_flag = 0;
  int temperature_flag = 0;

  //1代表彩色视频，0代表黑白视频，即mask  创建CvVideoWriter对象并分配空间
  VideoWriter wrVideo;
  string filename = "/home/pi/lepton/Lepton_image/video/lepton.avi";
  int codec = CV_FOURCC('M','J','P','G');
  double fps = 27;
  int count =0;

  void run();

public slots:
  void performFFC();
  void imagecapture();
  void switchMode();
  void changeTemperature();
  void changeBoxMode();

signals:
  void updateText1(QString);
  void updateText2(QString);
  void updateImage_IR(QImage);
  void NetImage_160_IR(QImage);


private:

  QImage Image_rgb;

  //164*60
  uint8_t result[4][PACKET_SIZE*PACKETS_PER_SEGMENT];

  QString maxTemperature_all;
  QString minTemperature_all;

  int serialPortfd;
  vector<Target> targets;

  void readIRimg();
  Mat readIRimg_lepton2_5();
  Mat QImage2cvMat(QImage image);
  QImage cvMat2QImage(const Mat& mat);
  void Identify(Mat& frame,double Threshold,double Threshold_maxVal);
  void Identify_Face(Mat& frame,double Threshold,double Threshold_maxVal);
  void IdentifyYolofastest(Mat& frame, Mat& frameDisplay, ncnn::Allocator*upa, ncnn::Allocator*pa);
  void TemperatureAbnormalAlarm(Mat& frame,uint16_t txt_x, uint16_t txt_y, uint16_t txt_w, uint16_t txt_h, float ROItemperature);
  void GetTemperature(Mat& frame, Mat& frameDisplay);
  void Acquisition(Mat& cvImage);
  void RgbAndIRAcquisition_IR(Mat& frame);
  void VideoAcquisition(Mat& frame);
  void Send(vector<Rect>& rectangles);
  vector<Rect> MatchCircles(Rect& rect, vector<Vec3f>& circles);
  void modiRect(Rect& rect);
  void selectBox(Mat& frame);
  void MouseRoiTemperature(Mat& frame);
};

struct rawData{
    uint16_t *FrameBuffer;
    uint16_t MinValue = 65535;
    uint16_t MaxValue = 0;
};

struct IRImage{
    QImage MyQImage_160;
    Mat CvImage_160;
    QImage MyQImage_320;
    Mat CvImage_320;
};


float getROITemperature_quickSort(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int temperatureChoose);
float RealTempByFormula(float surroundTemp, float tempBySensor,float distance);
float getROITemperature(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int temperatureChoose);

#endif
