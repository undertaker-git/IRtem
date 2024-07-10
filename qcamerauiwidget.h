#ifndef QCAMERAUIWIDGET_H
#define QCAMERAUIWIDGET_H

#include <QWidget>
#include <QImage>
#include <QThread>
#include "cameraobject.h"

class QCameraUiWidget : public QWidget
{
    Q_OBJECT
public:
    explicit QCameraUiWidget(QWidget *parent = nullptr);
    ~QCameraUiWidget();

    //打开设备
    void open();
    //暂停
    void pause();
    //关闭设备
    void close();
protected:
    void paintEvent(QPaintEvent *);

signals:
    void sendOpenState(bool);

    void openCamera(QString);
    void closeCamera();

public slots:
    void setText(QString);
    QString getText();
    QString getTitle();
    void setTitle(QString);

    void setFlag(QString);
    QString getFlag();

    //接收图像并绘制
    void updateImage(QImage image);

private:
    QImage  image;
    QString m_text;//文本
    QString m_title;//deviceid
    QString m_flag;//标准

    CameraObject *m_CameraObject;
    QThread camThread;
};

#endif // QCAMERAUIWIDGET_H
