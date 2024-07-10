#ifndef CAMERAOBJECT_H
#define CAMERAOBJECT_H

#include <QObject>
#include <QCamera>
#include <QDebug>
#include <QCameraImageCapture>
#include <QCameraViewfinder>
#include <QTimer>


class CameraObject : public QObject
{
    Q_OBJECT
public:
    explicit CameraObject(QObject *parent = nullptr);

signals:
    void sendImage(QImage);

public slots:
    //禁止本地存储摄像头图片
    void  GetCapture();
    //图像转格式
    void CaptureImage(int  a,QImage image);

    void startCamera(QString);
    void stopCamera();

private:
    QTimer *  cammerTimer; //定时器
    QCamera  *Camera;   //摄像头
    QCameraImageCapture  *ImageCapture;  //捕捉器
    QImage  m_image;    //最终Lable显示的图片

};

#endif // CAMERAOBJECT_H
