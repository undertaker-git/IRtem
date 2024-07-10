#include "LeptonThread.h"





#include <iostream>
#include <vector>
#include <QString>
#include <QDebug>
#include <QPushButton>
#include <QWidget>

#include <string.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <MyLabel.h>

#include "yolo/yolo-fastest.h"
#include "net.h"



int changeBox_flag = 0;
float mouseTemperature = 0;
extern int sys_counter;
extern int pre_counter;
extern QPoint lastPoint;
extern QPoint endPoint;
//float tpe_add = 0.2;
extern float tpe_add;
extern candidate Candidate;
rawData RawData;
IRImage IrImg;
extern int RgbAndIrCaptureFlag;
extern int RgbAndIrFrame_num;
int TrackFrameCount = 0;
extern float compensateDistance;

LeptonThread::LeptonThread() : QThread()
{
}

LeptonThread::~LeptonThread() {
}

void LeptonThread::run()
{

    //open spi port
    SpiOpenPort(0);

    //串口初始化(ttyAMA0)
    if((serialPortfd = serialOpen("/dev/ttyAMA0",115200)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    }

    wiringPiSetup();
    pinMode(0,INPUT);
    pinMode(25,OUTPUT);
    digitalWrite(25,HIGH);
    delay(500);
    digitalWrite(25,LOW);

    closeAGCmode();
    openRADmode();

    rawData RawData;
    IRImage RawImage;

    //Mat cvImage(60, 80, CV_8UC3);
    //Mat cvImage160;
    //Mat cvImage320;
    QImage myQImage;

    ncnn::Allocator*upa=new ncnn::UnlockedPoolAllocator;
    ncnn::Allocator*pa =new ncnn::PoolAllocator;

    //int validFrame = 0;
    while(true)
    {
        // read the IRimg from lepton3.5 modue.
        readIRimg();

        maxTemperature_all = QString("%1").arg(float(RawData.MaxValue)/100.00 - 273.15+tpe_add);
        emit updateText1(maxTemperature_all);
        minTemperature_all = QString("%1").arg(float(RawData.MinValue)/100.00 - 273.15);
        emit updateText2(minTemperature_all);

        if(imagecapture_flag==1)
        {
           Acquisition(IrImg.CvImage_320);
        }
        imagecapture_flag=0;

        if(RgbAndIrCaptureFlag==1)
        {
           RgbAndIRAcquisition_IR(IrImg.CvImage_320);
        }
        RgbAndIrCaptureFlag=0;

        if(video_capture_flag==1)
        {
           VideoAcquisition(IrImg.CvImage_320);
        }

        // 双线性插值
        //resize(IrImg.CvImage_320, cvImage160, Size(160,120));
        //RawImage.CvImage_160 = cvImage160;
        //resize(cvImage, cvImage320, Size(320,240));
        //RawImage.CvImage_320 = cvImage320;

        // 2024 03  04  lixicai
        //imshow("cvImage_160",RawImage.CvImage_160);
        //imshow("cvImage_320",IrImg.CvImage_320);
        IdentifyYolofastest(IrImg.CvImage_160,IrImg.CvImage_320,upa,pa);

        myQImage = cvMat2QImage(IrImg.CvImage_320);

        //lets emit the signal for update
        //imshow("cvImage_160",cvImage_160);
        //emit updateImage_IR(IrImg.MyImage_320); //320 *240
        emit updateImage_IR(myQImage); //320 *240
        emit NetImage_160_IR(IrImg.MyQImage_160);

        sys_counter=sys_counter+1;
        if(sys_counter==255)
        {
            sys_counter=1;
        }
        usleep(1);
    }
    //finally, close SPI port just bcuz
    SpiClosePort(0);
}


void LeptonThread::IdentifyYolofastest(Mat& frame, Mat& frameDisplay, ncnn::Allocator*upa, ncnn::Allocator*pa)
{
    yoloFastest(targets, frame,frameDisplay,upa,pa);
    //GetTemperature(frame,frameDisplay);
    //imshow("cvImage_160",frameDisplay);
}


void LeptonThread::GetTemperature(Mat& frame, Mat& frameDisplay)
{
    float  Surround_temp=0.0;
    float  temperature=35.6;
    float  MajorityTemperature=35.6;
    static float  PriorTemperature=35.6;
    float  Temperature_kf=35.6;

    ofstream outfile;
    outfile.open("TempData.txt", ios::app);

    Surround_temp = Sht20_Test();

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 1, 2, 8);

    IplImage tmp = IplImage(frameDisplay);
    CvArr* arr = (CvArr*)&tmp;

    vector<float> x_last;
    vector<float> p_last;

    for(Target& target : targets)
    {
        cv::Rect rect = target.GetTargetRect();
        int    Area= rect.area();
        float  Distance= 14227.0*pow(rect.area(),-0.593);
        temperature = getROITemperature_quickSort(rect.x/2, rect.y/2, rect.width/2, rect.height/2, 2);//getROITemperature_quickSort
        temperature = RealTempByFormula(Surround_temp, temperature,Distance)+tpe_add;

        // 求数组中的众数
        target.TemperatureArray.append(temperature);
        target.addLocate();
        std::cout << " target.locate = " << target.getLocate()  << std::endl;
        //std::cout << "this->TemperatureArray.size() = " << target.TemperatureArray.size()  << std::endl;

        if((target.getLocate())==5)
        {
            MajorityTemperature = target.GetMajorityTemperature(temperature);
            PriorTemperature =MajorityTemperature;
        }

        //卡尔曼滤波
        if(targets.size()==1)
        {
            Temperature_kf= KalmanFilter_lepton(PriorTemperature,0.5,5.0);
            std::cout << "******************** Temperature_kf = " << Temperature_kf << std::endl;
        }

        //std::cout << "Area = " << Area << " Distance = " << Distance <<  " Temperature = " << temperature << " Temperature_kf = " << Temperature_kf << std::endl;
        //outfile << "Area = " << Area << " Distance = " << Distance << "  Temperature_formula = " << temperature << "    Temperature_kf = " << Temperature_kf << "    Temperature_surround = " << Surround_temp<< endl;


        string AREA = format("%.2f", PriorTemperature);
        string TEMPERATURE = format("%.2f", temperature);

        String text_display="("+ AREA +" , " + TEMPERATURE+ ")";
        char * display_text = new char[strlen(text_display.c_str())+1];
        strcpy(display_text, text_display.c_str());
        cvPutText(arr,display_text, Point2d(rect.x*2,2*(rect.y-3)),&font, CV_RGB(255,0,0));
    }
}


float getROITemperature_quickSort(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int temperatureChoose)
{
   //x, y, width, height of 80*60
   uint16_t roi_x = x;
   uint16_t roi_y = y;
   uint16_t roi_w = w;
   uint16_t roi_h = h;

   uint32_t sumValue_roi = 0;
   uint16_t minValue_roi = 65535;
   uint16_t maxValue_roi = 0;
   vector<uint16_t> MeasurementVector;
   int k = 0;

   for(int j=roi_y;j<roi_y+roi_h;j++)
   {
       for(int i=(roi_x+2);i<(roi_x+roi_w+2);i++)
       {
           MeasurementVector.push_back(RawData.FrameBuffer[i+j*PACKET_SIZE_UINT16]);
           k += 1;
       }
   }
   //quickSort
   sort(MeasurementVector.begin(), MeasurementVector.end());
   maxValue_roi = MeasurementVector[k-2];
   minValue_roi = MeasurementVector[1];
   MeasurementVector.clear();

//   std::cout << "MeasurementVector = " << std::endl;
//   for(int n=0;n<k;n++)
//   {
//        std::cout << MeasurementVector[n] << std::endl;
//   }

   float avgTemperature_roi = float(sumValue_roi/(roi_w*roi_h))/100 - 273.15;
   float maxTemperature_roi = float(maxValue_roi)/100 - 273.15;
   float minTemperature_roi = float(minValue_roi)/100 - 273.15;
   float Temperature_r;
   switch(temperatureChoose)
   {
   case 1:
       Temperature_r = avgTemperature_roi;
       break;
   case 2:
       Temperature_r = maxTemperature_roi;
       break;
   case 3:
       Temperature_r = minTemperature_roi;
       break;
   }
   //std::cout << Temperature_r << std::endl;
   return Temperature_r;
}


float getROITemperature(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int temperatureChoose)
{
   //x, y, width, height of 80*60
   uint16_t roi_x = x;
   uint16_t roi_y = y;
   uint16_t roi_w = w;
   uint16_t roi_h = h;

   uint32_t sumValue_roi = 0;
   uint16_t minValue_roi = 65535;
   uint16_t maxValue_roi = 0;
   for(int j=roi_y;j<roi_y+roi_h;j++)
   {
       uint16_t tmp[80];
       int k = 0;
       for(int i=0;i<(PACKET_SIZE_UINT16);i++)
       {
           if(i%PACKET_SIZE_UINT16<2)
               continue;
           tmp[k] = RawData.FrameBuffer[i+j*PACKET_SIZE_UINT16];
           k += 1;
           //qDebug() << tmp[k] ;
       }
       for(int i=roi_x;i<roi_x+roi_w;i++)
       {
           sumValue_roi += uint32_t(tmp[i]);
           if(maxValue_roi<tmp[i])
               maxValue_roi = tmp[i];
           if(minValue_roi>tmp[i])
               minValue_roi = tmp[i];
       }
   }

   float avgTemperature_roi = float(sumValue_roi/(roi_w*roi_h))/100 - 273.15;
   float maxTemperature_roi = float(maxValue_roi)/100 - 273.15;
   float minTemperature_roi = float(minValue_roi)/100 - 273.15;
   float Temperature_r;
   switch(temperatureChoose)
   {
   case 1:
       Temperature_r = avgTemperature_roi;
       break;
   case 2:
       Temperature_r = maxTemperature_roi;
       break;
   case 3:
       Temperature_r = minTemperature_roi;
       break;
   }
   return Temperature_r;
}


float  RealTempByFormula(float surroundTemp, float tempBySensor, float distance)
{
    float realTemp;
    float epsilon = 0.98;  // human surface emissivity
    float tao = 1;         // atmosphere transmittance
    float n = 4.09;        // defined by wave length

    realTemp = pow(1/epsilon*(1/tao*pow(tempBySensor, n)-(1-epsilon)*pow(surroundTemp, n)), 1/n);

    //200 ~ 400
    if((distance >= compensateDistance) && (distance <= 2*compensateDistance))
    {
        //realTemp = distance*0.0061+realTemp;
        realTemp = 1.1+realTemp;
    }
    //400~800
    else if(distance > 2*compensateDistance)
    {
        realTemp = 2.5+realTemp;
    }

    return realTemp;
}


//the IR capture function
void LeptonThread::readIRimg()
{
    QImage myImageQ_160 = QImage(160, 120, QImage::Format_RGB888);
    QImage myImageQ_320 = QImage(320, 240, QImage::Format_RGB888);
    uint16_t *frameBuffer;

    //read data packets from lepton over SPI
    int resets = 0;
    int NumberRead=0;
    int segmentNumber;
    //PACKETS_PER_FRAME=60  一行读一次
    for(int segment=1;segment<=4;segment++)
    {
        for(int j=0;j<PACKETS_PER_SEGMENT;j++)
        {
            //if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
            NumberRead=read(spi_cs0_fd, (*(result+segment-1))+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
            int packetNumber = result[segment-1][j*PACKET_SIZE+1];
            //std::cout<<" segment: "<< segment << "  packetNumber: "<<packetNumber<<std::endl;
            if(packetNumber != j)
            {
                j = -1;
                resets += 1;
                usleep(300);//old data  1000
                //qDebug() << "packetNumber != j! " << resets;
                //Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
                //By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
                if(resets == 750)
                {
                    SpiClosePort(0);
                    usleep(7500);
                    SpiOpenPort(0);
                    //qDebug() << "resets == 750!" << resets;
                }
            }
            //check segment number
            if(packetNumber == VALID_TTT_PACKET)
            {
                segmentNumber = (result[segment-1][VALID_TTT_PACKET*PACKET_SIZE]>>4);
            }
        }
        if(segmentNumber == 1)
        {
            memcpy(result[0], result[segment-1], sizeof(result[segment-1]));
            segment = 1;
        }
        else if(segmentNumber!=segment)
        {
            segment = 0;
        }
    }
    //****************************************************
    if(resets >= 30)
    {
        //qDebug() << "done reading, resets: " << resets;
    }
    //*************************************************************************
    frameBuffer = (uint16_t *)result;
    RawData.FrameBuffer = frameBuffer;
    int row, column;
    uint16_t value;
    uint16_t minValue = 65535;
    uint16_t maxValue = 0;
    double Threshold = 195;//180
    double Threshold_maxVal = 255;

    // 160 X 120
    for(int segment=1;segment<=4;segment++)
    {
        for(int i=0;i<SEGMENT_SIZE_UINT16;i++)
        {
            //skip the first 2 uint16_t's of every packet, they're 4 header bytes
            if(i % PACKET_SIZE_UINT16 < 2)
            {
                continue;
            }

            //flip the MSB and LSB at the last second
            int temp = result[segment-1][i*2];
            result[segment-1][i*2] = result[segment-1][i*2+1];
            result[segment-1][i*2+1] = temp;

            value = frameBuffer[(segment-1)*SEGMENT_SIZE_UINT16+i];
            //std::cout<<"value: "<<value<<std::endl;
            if(value > maxValue) {
                maxValue = value;
            }
            if(value < minValue) {
                minValue = value;
            }
        }
    }
    //*********************************************************
    //std::cout<<"max="<<maxValue<<std::endl;
    //std::cout<<"min="<<minValue<<std::endl;
    //std::cout<<_AGCState<<std::endl;
    RawData.MaxValue = maxValue;
    RawData.MinValue = minValue;

    float diff = maxValue - minValue;
    float scale = 255/diff;
    QRgb color;

    Mat cvImage_160(120, 160, CV_8UC3);
    Mat cvImage_320(240, 320, CV_8UC3);

    for(int segment=1;segment<=4;segment++)
    {
        for(int i=0;i<SEGMENT_SIZE_UINT16;i++)
        {
            if(i % PACKET_SIZE_UINT16 < 2)
            {
                continue;
            }
            value = (frameBuffer[(segment-1)*SEGMENT_SIZE_UINT16+i] - minValue) * scale;
            if(value > 255)
            {
                value = 0;
            }
            if(value < 0)
            {
                value = 0;
            }

            //std::cout<<value<<std::endl;
            if(i % PACKET_SIZE < 82){
                column = (i % PACKET_SIZE_UINT16 ) - 2;      //160(previous 80)
            }
            else {
                column = (i % PACKET_SIZE_UINT16 ) - 2 + 80; //160(next 80)
            }
            row = (i / PACKET_SIZE) + (segment - 1) * 30;            //120
            if(mode_flag == 0)
            {
                const int *colormap = colormap_rainbow; //colormap_grayscale;colormap_rainbow
                color = qRgb(colormap[3*value], colormap[3*value+1], colormap[3*value+2]);
                myImageQ_160.setPixel(column, row, color);
            }
            else
            {
                const int *colormap = colormap_grayscale;
                cvImage_160.at<Vec3b>(row, column)[0] = colormap[3*value];
                cvImage_160.at<Vec3b>(row, column)[1] = colormap[3*value+1];
                cvImage_160.at<Vec3b>(row, column)[2] = colormap[3*value+2];
            }
        }
    }

    if(mode_flag == 0)
        cvImage_160 = QImage2cvMat(myImageQ_160);

    resize(cvImage_160, cvImage_320, Size(320,240),2,2);
    //imshow("cvImage_160",cvImage_160);
    GaussianBlur(cvImage_320, cvImage_320, Size(3, 3), 0.65, 0.65);

    if(changeBox_flag==1)
    {
      MouseRoiTemperature(cvImage_320);
      //selectBox(cvImage_320);
    }
    else
        ;//Identify(cvImage_320,Threshold,Threshold_maxVal);

    //imshow("cvImage_160",cvImage_160);
    //imshow("cvImage_320",cvImage_320);

    myImageQ_320 = cvMat2QImage(cvImage_320);
    myImageQ_160 = cvMat2QImage(cvImage_160);
    //cvImage_320 = QImage2cvMat(myImageQ_320);
    //cvImage_160 = QImage2cvMat(myImageQ_160);

    IrImg.MyQImage_160 = myImageQ_160;
    IrImg.CvImage_160  = cvImage_160;
    IrImg.MyQImage_320 = myImageQ_320;
    IrImg.CvImage_320  = cvImage_320;

}


//the IR capture function  lepton2.5
Mat LeptonThread::readIRimg_lepton2_5()
{
//    //read data packets from lepton over SPI
//     int resets = 0;
//     for(int j=0;j<PACKETS_PER_FRAME;j++) {
//         //if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
//         read(spi_cs0_fd, result+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
//         int packetNumber = result[j*PACKET_SIZE+1];
//         //std::cout<<"packetNumber: "<<packetNumber<<" j:"<<j<<std::endl;
//         if(packetNumber != j) {
//             j = -1;
//             resets += 1;
//             usleep(1000);
//             //Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
//             //By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
//             if(resets == 750) {
//                 SpiClosePort(0);
//                 std::cout <<"resets == 750"<<std::endl;
//                 usleep(750000);
//                 SpiOpenPort(0);
//                 camReboot();
//             }
//         }
//     }
//     if(resets >= 30) {
//         //qDebug() << "done reading, resets: " << resets;
//     }
//     frameBuffer = (uint16_t *)result;
//     int row, column;
//     uint16_t value;
//     uint16_t minValue = 65535;
//     uint16_t maxValue = 0;
//     double Threshold = 180;
//     double Threshold_maxVal = 255;

//     for(int i=0;i<FRAME_SIZE_UINT16;i++) {
//         //skip the first 2 uint16_t's of every packet, they're 4 header bytes
//         if(i % PACKET_SIZE_UINT16 < 2) {
//             continue;
//         }

//         //flip the MSB and LSB at the last second
//         int temp = result[i*2];
//         result[i*2] = result[i*2+1];
//         result[i*2+1] = temp;

//         value = frameBuffer[i];
//         if(value > maxValue) {
//             maxValue = value;
//         }
//         if(value < minValue) {
//             minValue = value;
//         }
//         column = i % PACKET_SIZE_UINT16 - 2;
//         row = i / PACKET_SIZE_UINT16 ;
//     }

//     //std::cout<<"max="<<maxValue<<std::endl;
//     //std::cout<<"max_celcius="<<((maxValue/50)-40)<<std::endl;
//     //std::cout<<"min="<<minValue<<std::endl;
//     //std::cout<<_AGCState<<std::endl;

//     float diff = maxValue - minValue;
//     float scale = 255/diff;
//     //QRgb color;

     Mat cvImage(60, 80, CV_8UC3);

//     for(int i=0;i<FRAME_SIZE_UINT16;i++) {
//         if(i % PACKET_SIZE_UINT16 < 2) {
//             continue;
//         }
//         value = (frameBuffer[i] - minValue) * scale;
//         if(value > 255) value = 0;
//         if(value < 0) value = 0;

//         //std::cout<<value<<std::endl;

//         const int *colormap = colormap_grayscale;
//         //color = qRgb(colormap[3*value], colormap[3*value+1], colormap[3*value+2]);
//         column = (i % PACKET_SIZE_UINT16 ) - 2;
//         row = i / PACKET_SIZE_UINT16;
//         //myImage.setPixel(column, row, color);
//         cvImage.at<Vec3b>(row, column)[0] = colormap[3*value];   //R
//         cvImage.at<Vec3b>(row, column)[1] = colormap[3*value+1]; //G
//         cvImage.at<Vec3b>(row, column)[2] = colormap[3*value+2]; //B
//     }
//     // 双线性插值
//     resize(cvImage, cvImage, Size(320,240));
     //std::cout<<value<<std::endl;
     return cvImage;
}


//******************performFFC************************************
void LeptonThread::performFFC(){
    lepton_perform_ffc();
}

//******************************************************
void LeptonThread::imagecapture()
{
    imagecapture_flag=1;
    image_num = image_num+1;
    //camReboot();
}


Mat LeptonThread::QImage2cvMat(QImage image)
{
    cv::Mat mat;
    //qDebug() << image.format();
    switch(image.format())
    {
    case QImage::Format_ARGB32:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32_Premultiplied:
        mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
        break;
    case QImage::Format_RGB888:
        mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
        cv::cvtColor(mat, mat, CV_BGR2RGB);
        break;
    case QImage::Format_Indexed8:
        mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
        break;
    default:
        break;
    }
    return mat;
}

QImage LeptonThread::cvMat2QImage(const Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

void LeptonThread::Identify(Mat& frame,double Threshold,double Threshold_maxVal){

    Mat gray_image;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8);

    /***************************************热度区域检测*******************************************/
    Mat binary_image;
    Mat dilate_img;

    cvtColor(frame, gray_image, CV_BGR2GRAY);       //灰度处理
    //qDebug()<<"Threshold = " <<Threshold;
    //threshold(gray_image, binary_image, 90, 255, CV_THRESH_BINARY);//二值化 100 255
    threshold(gray_image, binary_image, Threshold, Threshold_maxVal, CV_THRESH_BINARY);//二值化 100 255
    Mat de_noise = binary_image.clone();
    medianBlur(binary_image, de_noise, 5);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//膨胀
    dilate(de_noise, dilate_img,element);
    //imshow("dilate_img",dilate_img);

    //检测连通域，每一个连通域以一系列的点表示，FindContours方法只能得到第一个域
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //CV_RETR_EXTERNAL只检测外部轮廓，可根据自身需求进行调整
    findContours(dilate_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //qDebug() << "contoursSize= "<<contours.size();
    vector<Rect> rectangles;
    for (int index = 0; index < contours.size(); index = hierarchy[index][0])
    {
        Rect rect = boundingRect(contours[index]);//检测外轮廓
        //skip rects too small at the bottom of img
        if((rect.area() <= 450 || min(rect.width, rect.height) <= 90) && rect.y > ORIGROW * 2 / 3){
            continue;
        }
        if((rect.width<=1.6*rect.height)&&((rect.height<=2.6*rect.width))&&((rect.width>=10)))
        //if(rect.width>=5)
        {
            //float temperature = SetROI_user((LEP_UINT16)rect.x/2,(LEP_UINT16)rect.y/2,(LEP_UINT16)(rect.x/2+rect.width/2),(LEP_UINT16)(rect.y/2+rect.height/2));
            float ROItemperature = getROITemperature(rect.x, rect.y, rect.width, rect.height, temperature_flag);
            if(temperature_flag==1)
                std::cout << "average ";
            else if(temperature_flag==2)
                std::cout << "max ";
            else if(temperature_flag==3)
                std::cout << "min ";
            if(temperature_flag)
                std::cout << "autodetected_temperature = " << ROItemperature << std::endl;
            rectangles.push_back(rect);
            rectangle(frame, rect, Scalar(0, 0, 255), 2);//对外轮廓加矩形框
            IplImage tmp = IplImage(frame);
            CvArr* arr = (CvArr*)&tmp;
            String WIDTH=to_string( (long long)rect.width );
            String HEIGTH=to_string( (long long)rect.height );
            String TEMPER=to_string( (float)ROItemperature );
            TEMPER=TEMPER.substr(0,TEMPER.size()-5);
            //String text_play="("+WIDTH+","+HEIGTH+")";
            String text_play="("+TEMPER+")";
            char * play_text = new char[strlen(text_play.c_str())+1];
            strcpy(play_text, text_play.c_str());
            //cvPutText(arr, to_string((long long)rect.width).c_str(), Point2d(rect.x,rect.y),&font, CV_RGB(255,255,0));
            cvPutText(arr,play_text, Point2d(rect.x,rect.y),&font, CV_RGB(255,255,0));

            if( ROItemperature>37.5 )
            {
                ;//TemperatureAbnormalAlarm(frame,rect.x, rect.y, rect.width, rect.height,ROItemperature);
            }
        }
    }
    Send(rectangles);
}

//温度报警
void LeptonThread::TemperatureAbnormalAlarm(Mat& frame,uint16_t txt_x, uint16_t txt_y, uint16_t txt_w, uint16_t txt_h, float ROItemperature)
 {
    Mat gray_image;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8);

    QDateTime DateTime(QDateTime::currentDateTime());
    QString qStr = DateTime.toString("yy-MM-dd_hh-mm-ss");
    qDebug() << "DateTime = " <<qStr;
    string ShootTime = qStr.toStdString();

    String Img_Name1 = "/home/pi/lixicai/Lepton_image_V8/image/Temperature_anomaly/image_IR/" + ShootTime +".jpg";
    String Img_Name2 = "/home/pi/lixicai/Lepton_image_V8/image/Temperature_anomaly/image_IR/Temp/" + to_string(image_num)+".jpg";
    if(mode_flag == 1)
    {
        cvtColor(frame, gray_image, CV_BGR2GRAY);
        GaussianBlur(gray_image, gray_image, Size(3, 3), 0.65, 0.65);
    }
    else
        GaussianBlur(frame, gray_image, Size(3, 3), 0.65, 0.65);

    String TEMPER=to_string((long long)ROItemperature);
    //String text_play="("+WIDTH+","+HEIGTH+")";
    String text_play="("+TEMPER+")";
    char * play_text = new char[strlen(text_play.c_str())+1];
    strcpy(play_text, text_play.c_str());
    //cvPutText(arr, to_string((long long)rect.width).c_str(), Point2d(rect.x,rect.y),&font, CV_RGB(255,255,0));
    IplImage tmp = IplImage(gray_image);
    CvArr* arr = (CvArr*)&tmp;
    cvPutText(arr,play_text, Point2d(txt_x,txt_y),&font, CV_RGB(255,255,0));
    rectangle (gray_image, Point(txt_x, txt_y), Point(txt_x+txt_w, txt_y+txt_h), Scalar(0, 255, 255), 2, 8, 0);//黄色矩形镶边

    //保存图片
    imwrite(Img_Name1,gray_image);
    imwrite(Img_Name2,gray_image);
    //Mat reimg = imread(Img_Name1, 0);
    imshow("HEIGTH",gray_image);
}

void LeptonThread::Identify_Face(Mat& frame,double Threshold,double Threshold_maxVal){

    Mat gray_image;
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8);

    /***************************************热度区域检测*******************************************/
    Mat binary_image;
    Mat dilate_img;

    cvtColor(frame, gray_image, CV_BGR2GRAY);       //灰度处理
    //qDebug()<<"Threshold = " <<Threshold;
    //threshold(gray_image, binary_image, 90, 255, CV_THRESH_BINARY);//二值化 100 255
    threshold(gray_image, binary_image, Threshold, Threshold_maxVal, CV_THRESH_BINARY);//二值化 100 255
    Mat de_noise = binary_image.clone();
    medianBlur(binary_image, de_noise, 5);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));//膨胀
    dilate(de_noise, dilate_img,element);
    //imshow("dilate_img",dilate_img);

    //检测连通域，每一个连通域以一系列的点表示，FindContours方法只能得到第一个域
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    //CV_RETR_EXTERNAL只检测外部轮廓，可根据自身需求进行调整
    findContours(dilate_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //qDebug() << "contoursSize= "<<contours.size();
    vector<Rect> rectangles;
    for (int index = 0; index < contours.size(); index = hierarchy[index][0])
    {
        Rect rect = boundingRect(contours[index]);//检测外轮廓
        //skip rects too small at the bottom of img
        if((rect.area() <= 400 || min(rect.width, rect.height) <= 20) && rect.y > ORIGROW * 2 / 3){
            continue;
        }
        //if((rect.width<=rect.height)&&((rect.height<=2*rect.width))&&((rect.width>=5)))
        if((rect.width<=1.6*rect.height)&&((rect.height<=2.6*rect.width))&&((rect.width>=5)))
        {
            //float temperature = SetROI_user((LEP_UINT16)rect.x/2,(LEP_UINT16)rect.y/2,(LEP_UINT16)(rect.x/2+rect.width/2),(LEP_UINT16)(rect.y/2+rect.height/2));
            float ROItemperature = getROITemperature(rect.x, rect.y, rect.width, rect.height, temperature_flag);
            if(temperature_flag==1)
                std::cout << "average ";
            else if(temperature_flag==2)
                std::cout << "max ";
            else if(temperature_flag==3)
                std::cout << "min ";
            if(temperature_flag)
                std::cout << "autodetected_temperature = " << ROItemperature << std::endl;
            rectangles.push_back(rect);
            rectangle(frame, rect, Scalar(0, 0, 255), 2);//对外轮廓加矩形框
            IplImage tmp = IplImage(frame);
            CvArr* arr = (CvArr*)&tmp;
            String WIDTH=to_string((long long)rect.width);
            String HEIGTH=to_string((long long)rect.height);
            String TEMPER=to_string((float)ROItemperature);
            //String text_play="("+WIDTH+","+HEIGTH+")";
            String text_play="("+TEMPER+")";
            char * play_text = new char[strlen(text_play.c_str())+1];
            strcpy(play_text, text_play.c_str());
            //cvPutText(arr, to_string((long long)rect.width).c_str(), Point2d(rect.x,rect.y),&font, CV_RGB(255,255,0));
            cvPutText(arr,play_text, Point2d(rect.x,rect.y),&font, CV_RGB(255,255,0));
        }
    }
    Send(rectangles);
}


void LeptonThread::Acquisition(Mat& frame){
    Mat gray_image;
    string Img_Name = "/home/pi/Pictures/image/IR/" +to_string(image_num)+".jpg";
    if(mode_flag == 1)
    {
        cvtColor(frame, gray_image, CV_BGR2GRAY);
        GaussianBlur(gray_image, gray_image, Size(3, 3), 0.65, 0.65);
    }
    else
        GaussianBlur(frame, gray_image, Size(3, 3), 0.65, 0.65);
    //保存图片
    imwrite(Img_Name,gray_image);
}



void LeptonThread::RgbAndIRAcquisition_IR(Mat& frame)
{
    //cout << "test"<< RgbAndIrFrame_num<<endl;
    Mat gray_image;
    string Img_Name = "/home/pi/Pictures/image/RgbAndIR/IR/" +to_string(RgbAndIrFrame_num)+".jpg";
    if(mode_flag == 1)
    {
        cvtColor(frame, gray_image, CV_BGR2GRAY);
        GaussianBlur(gray_image, gray_image, Size(3, 3), 0.65, 0.65);
    }
    else
        GaussianBlur(frame, gray_image, Size(3, 3), 0.65, 0.65);
    //保存图片
    imwrite(Img_Name,gray_image);
}



int countF = 0;
void LeptonThread::VideoAcquisition(Mat& frame){
    Mat gray_image;
    Mat dst_img;

    if(mode_flag == 1)
    {
        cvtColor(frame, gray_image, CV_BGR2GRAY);
        GaussianBlur(gray_image, gray_image, Size(3, 3), 0.65, 0.65);
        // 双线性插值
        resize(gray_image, dst_img, Size(320,240));

    }
    else
    {
        GaussianBlur(frame, frame, Size(3, 3), 0.65, 0.65);
        resize(frame, dst_img, Size(320, 240));
    }
    countF++;
    qDebug() << "write frame: " << countF;
}


vector<Rect> LeptonThread::MatchCircles(Rect& rect, vector<Vec3f>& circles){
    vector<Rect> rects;
    for (size_t i = 0; i < circles.size(); i++){
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        if(center.x - radius * 0.5 > rect.x
            && center.y - radius * 0.5 > rect.y
            && center.x + radius * 0.5 < rect.x + rect.width
            && center.y + radius * 0.5 < rect.y + rect.height){
            int x = center.x - radius * 1.3 > 0 ? center.x - radius * 1.3 : 0;
            int y = center.y - radius * 1.3 > 0 ? center.y - radius * 1.3 : 0;
            int width = center.x + radius * 2.4 < 80 ? radius * 2.4 : 80;
            int height = center.y + radius * 2.4 < 60 ? radius * 2.4 : 60;
            Rect newrect(x, y, width, height);
            rects.push_back(newrect);
        }
    }
    return rects;
}

void LeptonThread::Send(vector<Rect>& rectangles){
    QString sendStr = "#";
    sendStr.append(QString::number(rectangles.size(), 10));
    QString space = " ";
    for(Rect rect : rectangles){
        modiRect(rect);
        sendStr.append(space);
        sendStr.append(QString::number(rect.x, 10));
        sendStr.append(space);
        sendStr.append(QString::number(rect.y, 10));
        sendStr.append(space);
        sendStr.append(QString::number(rect.width, 10));
        sendStr.append(space);
        sendStr.append(QString::number(rect.height, 10));
    }
    sendStr.append("*");
    sendStr.append(""+QString::number(count++, 10));
    sendStr.append("\r\n");
    //qDebug() << sendStr;
    serialFlush(serialPortfd);
    serialPuts(serialPortfd, sendStr.toLatin1().data());
}

void LeptonThread::modiRect(Rect& rect){
    rect.x = rect.x * 2.6 + 224 - 40;
    rect.y = rect.y * 2.6 + 96 - 40;
    rect.width = rect.width * 2.6 + 80;
    rect.height = rect.height * 2.6 + 80;
    rect.x = max(0, min(rect.x, MODICOL));
    rect.y = max(0, min(rect.y, MODIROW));
    rect.width = rect.x + rect.width >= MODICOL ? MODICOL - rect.x - 1 : rect.width;
    rect.height = rect.y + rect.height >= MODIROW ? MODIROW - rect.y - 1 : rect.height;
}

//float LeptonThread::getROITemperature(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int temperatureChoose)
//{
//   //x, y, width, height of 160*120
//   uint16_t roi_x = x/2;
//   uint16_t roi_y = y/2;
//   uint16_t roi_w = w/2;
//   uint16_t roi_h = h/2;

//   uint32_t sumValue_roi = 0;
//   uint16_t minValue_roi = 65535;
//   uint16_t maxValue_roi = 0;
//   for(int j=roi_y;j<roi_y+roi_h;j++)
//   {
//       uint16_t tmp[160];
//       int k = 0;
//       for(int i=0;i<(PACKET_SIZE_UINT16*2);i++)
//       {
//           if(i%PACKET_SIZE_UINT16<2)
//               continue;
//           tmp[k] = RawData.FrameBuffer[i+j*PACKET_SIZE_UINT16*2];
//           k += 1;
//       }
//       for(int i=roi_x;i<roi_x+roi_w;i++)
//       {
//           sumValue_roi += uint32_t(tmp[i]);
//           if(maxValue_roi<tmp[i])
//               maxValue_roi = tmp[i];
//           if(minValue_roi>tmp[i])
//               minValue_roi = tmp[i];
//       }
//   }

//   float avgTemperature_roi = float(sumValue_roi/(roi_w*roi_h))/100 - 273.15;
//   float maxTemperature_roi = float(maxValue_roi)/100 - 273.15;
//   float minTemperature_roi = float(minValue_roi)/100 - 273.15;
//   float Temperature_r;
//   switch(temperatureChoose)
//   {
//   case 1:
//       Temperature_r = avgTemperature_roi;
//       break;
//   case 2:
//       Temperature_r = maxTemperature_roi;
//       break;
//   case 3:
//       Temperature_r = minTemperature_roi;
//       break;
//   }
//   return Temperature_r+tpe_add;
//}

void LeptonThread::switchMode()
{
    if(mode_flag == 0)
        mode_flag = 1;
    else
        mode_flag = 0;
}

void LeptonThread::changeTemperature()
{
    QObject *object = QObject::sender();
    QPushButton *push_button = qobject_cast<QPushButton *>(object);
    QString object_name = push_button->objectName();

    if(object_name == "avgTemperature")
        temperature_flag = 1;
    else if(object_name == "maxTemperature")
        temperature_flag = 2;
    else if(object_name == "minTemperature")
        temperature_flag = 3;
}

void LeptonThread::changeBoxMode()
{
    if(changeBox_flag == 0)
    {
        changeBox_flag = 1;
        endPoint = lastPoint;
    }
    else
    {
        changeBox_flag = 0;
    }
}

void LeptonThread::selectBox(Mat& frame)
{
//    CvFont font;
//    cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8);
    if(temperature_flag)
    {
        uint16_t selected_x = lastPoint.x()/3*2;
        if(lastPoint.x()>endPoint.x())
            selected_x = endPoint.x()/3*2;
        uint16_t selected_y = lastPoint.y()/3*2;
        if(lastPoint.y()>endPoint.y())
            selected_y = endPoint.y()/3*2;
        uint16_t selected_w = abs(endPoint.x() - lastPoint.x())/3*2;
        uint16_t selected_h = abs(endPoint.y() - lastPoint.y())/3*2;
        if((endPoint.x()>0)&&(endPoint.x()<480)&&(endPoint.y())&&(endPoint.y()<360)&&selected_w&&selected_h)
        {
            //float mouseTemperature = getROITemperature(selected_x, selected_y, selected_w, selected_h, temperature_flag);
            mouseTemperature = getROITemperature(selected_x, selected_y, selected_w, selected_h, temperature_flag);
            if(temperature_flag==1)
                std::cout << "average ";
            else if(temperature_flag==2)
                std::cout << "max ";
            else if(temperature_flag==3)
                std::cout << "min ";
            //std::cout << "a=" << setprecision(2) << a <<endl;
            std::cout << "selected_temperature = " << mouseTemperature << std::endl;
//            IplImage tmp = IplImage(frame);
//            CvArr* arr = (CvArr*)&tmp;
//            String TEMPER=to_string((long long)mouseTemperature);
//            String text_play="("+TEMPER+")";
//            char * play_text = new char[strlen(text_play.c_str())+1];
//            strcpy(play_text, text_play.c_str());
//            std::cout << "(" << endPoint.x() << "," << endPoint.y() << ")"<<std::endl;
//            cvPutText(arr,play_text, Point2d(lastPoint.x()/3*2,lastPoint.y()/3*2),&font, CV_RGB(255,255,0));
        }

    }
}

void LeptonThread::MouseRoiTemperature(Mat& frame)
{
    if(temperature_flag)
    {
        uint16_t selected_x = Candidate.candidateXStart[Candidate.CandidateNumber]/3*2;
        uint16_t selected_y = Candidate.candidateYStart[Candidate.CandidateNumber]/3*2;
        uint16_t selected_w = abs(Candidate.candidateXEnd[Candidate.CandidateNumber] - Candidate.candidateXStart[Candidate.CandidateNumber])/3*2;
        uint16_t selected_h = abs(Candidate.candidateYEnd[Candidate.CandidateNumber] - Candidate.candidateYStart[Candidate.CandidateNumber])/3*2;
        if(Candidate.candidateXStart[Candidate.CandidateNumber]>Candidate.candidateXEnd[Candidate.CandidateNumber])
            selected_x = Candidate.candidateXEnd[Candidate.CandidateNumber]/3*2;
        if(Candidate.candidateYStart[Candidate.CandidateNumber]>Candidate.candidateYEnd[Candidate.CandidateNumber])
            selected_y = Candidate.candidateYEnd[Candidate.CandidateNumber]/3*2;
        if((Candidate.candidateXEnd[Candidate.CandidateNumber]>0)&&(Candidate.candidateXEnd[Candidate.CandidateNumber]<480)&&(Candidate.candidateYEnd[Candidate.CandidateNumber])&&(Candidate.candidateYEnd[Candidate.CandidateNumber]<360)&&selected_w&&selected_h)
        {
            mouseTemperature = getROITemperature(selected_x, selected_y, selected_w, selected_h, temperature_flag);
            Candidate.candidateTemperature[Candidate.CandidateNumber] = mouseTemperature;
            if(temperature_flag==1)
                std::cout << "average ";
            else if(temperature_flag==2)
                std::cout << "max ";
            else if(temperature_flag==3)
                std::cout << "min ";
            //std::cout << "a=" << setprecision(2) << a <<endl;
            std::cout << "selected_temperature = " << mouseTemperature << std::endl;
        }

    }
}
