#include "qcamerauiwidget.h"
#include <QPainter>
#include <QBrush>

QCameraUiWidget::QCameraUiWidget(QWidget *parent) : QWidget(parent)
{
    m_CameraObject = new CameraObject();
    m_CameraObject->moveToThread(&camThread);
    connect(&camThread, &QThread::finished, m_CameraObject, &QObject::deleteLater);

    connect(this,SIGNAL(openCamera(QString)),m_CameraObject,SLOT(startCamera(QString)));
    connect(this,SIGNAL(closeCamera()),m_CameraObject,SLOT(stopCamera()));
    connect(m_CameraObject,SIGNAL(sendImage(QImage)),this,SLOT(updateImage(QImage)));

    void openCamera(QString);
    void closeCamera();

    camThread.start();
}

QCameraUiWidget::~QCameraUiWidget()
{
    camThread.quit();
    camThread.wait();
}

void QCameraUiWidget::open()
{
    if(m_title!="0"){
          emit openCamera(m_title);
    }
}

void QCameraUiWidget::pause()
{
    emit closeCamera();
}

void QCameraUiWidget::close()
{
    emit closeCamera();
}

void QCameraUiWidget::setText(QString str)
{
    m_text = str;
    update();
}

void QCameraUiWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    if (image.isNull()) {
        image.fill(QColor(42,67,101));
        painter.setBrush(QBrush(QColor(42,67,101)));
        painter.drawRect(1,1,this->width()-2,this->height()-2);
        painter.translate(QPoint(this->width()/2,this->height()/2));
        painter.setPen(QPen(QColor("white")));
        painter.drawText(-10,0,m_text);
        return;
    }else{
        //    qDebug() << TIMEMS << "paintEvent" << image.width();

        painter.drawImage(this->rect(), image);
        //42,67,101
        painter.drawText(0,0,m_text);
    }
}

QString QCameraUiWidget::getText()
{
    return m_text;
}
void QCameraUiWidget::setFlag(QString str)
{
    m_flag = str;
}

QString QCameraUiWidget::getFlag()
{
    return m_flag;
}

void QCameraUiWidget::updateImage(QImage image)
{
    this->image = image;
    this->update();
}

QString QCameraUiWidget::getTitle()
{
    return m_title;
}
void QCameraUiWidget::setTitle(QString str)
{
    m_title = str;
}
