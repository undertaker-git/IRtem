#pragma execution_character_set("utf-8")
#include "cameraobject.h"
#include <QCameraInfo>

CameraObject::CameraObject(QObject *parent) : QObject(parent)
{
    cammerTimer = nullptr;
    Camera = nullptr;
    ImageCapture = nullptr;
}

//截图
void CameraObject::GetCapture()
{
    if(ImageCapture!=nullptr){
        ImageCapture->capture();  //本地不保存图片
    }

}
//获取一张图片
void CameraObject::CaptureImage(int a, QImage p_image)
{

    emit sendImage(p_image);
}

void CameraObject::startCamera(QString strs)
{
    qDebug()<<"startCamera=="<<strs;
    if(cammerTimer!=nullptr){
        delete cammerTimer;
    }
    if(Camera!=nullptr){
        delete Camera;
    }
    if(ImageCapture!=nullptr){
        delete ImageCapture;
    }

    QCameraInfo cameraInfo;

    // 获取所有可用摄像头信息列表
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

    // 遍历摄像头列表
    for (const QCameraInfo& camera : cameras) {
        qDebug() << "Camera Name:" << camera.deviceName();
        //        qDebug() << "Camera Description:" << camera.description();
        //        qDebug() << "Camera Orientation:" << camera.orientation();
        //        qDebug() << "Camera Position:" << camera.position();
        if(camera.deviceName() == strs){
            cameraInfo = camera;
        }
    }

    Camera = new QCamera(cameraInfo);  //摄像头
    ImageCapture = new QCameraImageCapture(Camera);//截图器

    //设置摄像头
    ImageCapture->setCaptureDestination(QCameraImageCapture::CaptureToBuffer);
    Camera->setCaptureMode(QCamera::CaptureStillImage);

    //定时捕获图像
    Camera->start();//开启摄像头

    cammerTimer = new QTimer();
    connect(cammerTimer,SIGNAL(timeout()),this,SLOT(GetCapture())); //建立信号和槽
    connect(ImageCapture, SIGNAL(imageCaptured(int, QImage)), this, SLOT(CaptureImage(int ,QImage)));
    cammerTimer->start(100);
}

void CameraObject::stopCamera()
{
    if(cammerTimer!=nullptr){
        cammerTimer->stop();
        delete cammerTimer;
        cammerTimer = nullptr;
    }
    if(Camera!=nullptr){
        Camera->stop();
        delete Camera;
        Camera = nullptr;
    }
    if(ImageCapture!=nullptr){
        delete ImageCapture;
        ImageCapture = nullptr;
    }
}
