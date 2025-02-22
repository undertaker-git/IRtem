﻿#pragma execution_character_set("utf-8")
#include "videopanel.h"
#include "qevent.h"
#include "qmenu.h"
#include "qlayout.h"
#include "qlabel.h"
#include "qtimer.h"
#include "qdebug.h"
#include "camerasetdialog.h"
#include <QMessageBox>



int RgbAndIrCaptureFlag = 0;
int RgbAndIrFrame_num =0;
int Shutdown_Flag=0;
float Surround_temp=0.0;
float compensateDistance=200;
float TemperatureOffsetADJ=0.0;

VideoPanel::VideoPanel(QWidget *parent) : QWidget(parent)
{
    userDataBasePri db;
    camera_info = userDataBasePri::queryAll("camera_info");

    m_widget = nullptr;
    this->initControl();
    this->initForm();
    this->initMenu();
    this->show_video_all();//显示
    QTimer::singleShot(1000, this, SLOT(play_video_all()));
    m_playnum = 0;
    m_timer = new QTimer();
    connect(m_timer,SIGNAL(timeout()),this,SLOT(mtimeout()));



}

VideoPanel::~VideoPanel()
{
    for(int i = 0;i < widgets.size();i++){
        QCameraUiWidget* t = widgets.at(i);
        t->close();
        delete t;
    }
    delete m_timer;
}

void VideoPanel::ReadFrame_RGB(){
    capture >> matimage;
    QImage img=cvMat2QImage(matimage);
//    QImage img((const uchar*)matimage.data, matimage.cols, matimage.rows, QImage::Format_RGB888);

    if(frame_flag==1)
    {
       RGBacqusition(matimage);
    }
    frame_flag=0;

    if(RgbAndIrCaptureFlag==1)
    {
       RgbAndIrAcquisition_RGB(matimage);
    }
    //RgbAndIrCaptureFlag=0;

    QPixmap pixmap = QPixmap::fromImage(img);
    pixmap.scaled(ui->temperatureBar->size(), Qt::KeepAspectRatio);
    ui->myLabel_Rgb->setScaledContents(true);
    ui->myLabel_Rgb->setPixmap(pixmap);

    uint16_t linesize  = (uint16_t )matimage.step;
    //emit Ffmpeg_Encoder_Encode_signals((uint8_t *)matimage.data, linesize ) ;

}
void MainWindow::NetImage_160_IR(QImage Image_160)
{
     imggray = QImage2cvMat(Image_160);
     send_frame((uint8_t*)imggray.data, (uint16_t)(imggray.cols*imggray.rows*3),2);

}

void MainWindow::NetReadFarme()
{
     capture >> matimage;
     QImage img((const uchar*)matimage.data, matimage.cols, matimage.rows, QImage::Format_RGB888);
     //ui->label->setPixmap(QPixmap::fromImage(img));
     uint16_t linesize  = (uint16_t )matimage.step;
     emit Ffmpeg_Encoder_Encode_signals((uint8_t *)matimage.data, linesize ) ;

}
void MainWindow::RGBcapture()
{
    frame_flag = 1;
    frame_num += 1;
   // cout << "test"<<endl;
}

void MainWindow::RGBacqusition(Mat image_in)
{
    String ImgRGB_Name = "/home/pi/Pictures/image/RGB/" +to_string(frame_num)+".jpg";
    imwrite(ImgRGB_Name, image_in);
}

QImage VideoPanel::cvMat2QImage(const Mat& mat)
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

void VideoPanel::on_RgbAndIRCapture_released()
{
    RgbAndIrCaptureFlag = 1;
    RgbAndIrFrame_num += 1;
    cout << "Capture : "<< RgbAndIrFrame_num<<endl;
}


void MainWindow::RgbAndIrAcquisition_RGB(Mat image_in)
{
    String ImgRGB_Name = "/home/pi/Pictures/image/RgbAndIR/RGB/" +to_string(RgbAndIrFrame_num)+".jpg";
    imwrite(ImgRGB_Name, image_in);
}


void VideoPanel::closeAllVideo()
{
    for (int i = 0; i < widgets.size(); i++) {
        widgets.at(i)->close();
    }
}

bool VideoPanel::eventFilter(QObject *watched, QEvent *event)
{
    if (event->type() == QEvent::MouseButtonDblClick) {
        QLabel *widget = (QLabel *) watched;
        if (!videoMax) {
            videoMax = true;
            hide_video_all();
            gridLayout->addWidget(widget, 0, 0);
            widget->setVisible(true);
        } else {
            videoMax = false;
            show_video_all();
        }
        widget->setFocus();
    } else if (event->type() == QEvent::MouseButtonPress) {
        QMouseEvent *mouseEvent = (QMouseEvent *)event;
        if (mouseEvent->button() == Qt::RightButton) {
            m_widget = (QCameraUiWidget *) watched;
            videoMenu->exec(QCursor::pos());
        }
    }
    return QWidget::eventFilter(watched, event);
}

void VideoPanel::showEvent(QShowEvent *event)
{
    // m_timer->start(1000);//屏蔽自动开启
}

QSize VideoPanel::sizeHint() const
{
    return QSize(800, 600);
}

QSize VideoPanel::minimumSizeHint() const
{
    return QSize(80, 60);
}

void VideoPanel::initControl()
{
    gridLayout = new QGridLayout;
    gridLayout->setSpacing(1);
    gridLayout->setMargin(0);
    gridLayout->setObjectName("gridLayout");
    this->setLayout(gridLayout);
}

void VideoPanel::initForm()
{
    //设置样式表
    QStringList qss;
    qss.append("QFrame{border:2px solid #000000;}");
    qss.append("QLabel{font:75 25px;color:#F0F0F0;border:2px solid #AAAAAA;background:#000000;}");
    qss.append("QLabel:focus{border:2px solid #00BB9E;background:#555555;}");
    this->setStyleSheet(qss.join(""));

    videoMax = false;
    videoCount = 2;
    videoType = "1_9";
    for (int i = 0; i < videoCount; i++) {
        QCameraUiWidget *s_widget;
        s_widget = new QCameraUiWidget;
        connect(s_widget, SIGNAL(sendOpenState(bool)), this, SLOT(acceptOpenState(bool)));
        s_widget->setTitle(QString::number(i+1));//界面ID
        for(int fx = 0;fx < camera_info.size();fx++){
            if(camera_info.at(fx).at(0) == QString("flag%1").arg(i + 1)){
                s_widget->setTitle(camera_info.at(fx).at(1));
            }
        }

        s_widget->setObjectName(QString("video%1").arg(i + 1));
        s_widget->installEventFilter(this);
        s_widget->setFocusPolicy(Qt::StrongFocus);
        //二选一可以选择显示文字,也可以选择显示背景图片
        s_widget->setText(QString("通道 %1").arg(i + 1));
        s_widget->setFlag(QString("flag%1").arg(i + 1));

        widgets.append(s_widget);
    }
}

void VideoPanel::initMenu()
{
    videoMenu = new QMenu(this);

    actionOpen = new QAction("打开摄像头", videoMenu);
    connect(actionOpen, SIGNAL(triggered(bool)), this, SLOT(openCamera()));
    videoMenu->addAction(actionOpen);

    actionClose = new QAction("关闭摄像头", videoMenu);
    connect(actionClose, SIGNAL(triggered(bool)), this, SLOT(closeCameraInfo()));
    videoMenu->addAction(actionClose);

    actionSet = new QAction("摄像头设置", videoMenu);
    connect(actionSet, SIGNAL(triggered(bool)), this, SLOT(setCameraInfo()));
    videoMenu->addAction(actionSet);

    actionOpenAll = new QAction("打开全部", videoMenu);
    connect(actionOpenAll, SIGNAL(triggered(bool)), this, SLOT(openCameraAll()));
    videoMenu->addAction(actionOpenAll);

    videoMenu->addSeparator();

    QMenu *menu4 = videoMenu->addMenu("切换到4画面");
    menu4->addAction("通道1-通道4", this, SLOT(show_video_4()));
    menu4->addAction("通道5-通道8", this, SLOT(show_video_4()));
    menu4->addAction("通道9-通道12", this, SLOT(show_video_4()));
    menu4->addAction("通道13-通道16", this, SLOT(show_video_4()));

    QMenu *menu6 = videoMenu->addMenu("切换到6画面");
    menu6->addAction("通道1-通道6", this, SLOT(show_video_6()));
    menu6->addAction("通道6-通道11", this, SLOT(show_video_6()));
    menu6->addAction("通道11-通道16", this, SLOT(show_video_6()));

    QMenu *menu8 = videoMenu->addMenu("切换到8画面");
    menu8->addAction("通道1-通道8", this, SLOT(show_video_8()));
    menu8->addAction("通道9-通道16", this, SLOT(show_video_8()));

    QMenu *menu9 = videoMenu->addMenu("切换到9画面");
    menu9->addAction("通道1-通道9", this, SLOT(show_video_9()));
    menu9->addAction("通道8-通道16", this, SLOT(show_video_9()));

    QMenu *menu13 = videoMenu->addMenu("切换到13画面");
    menu13->addAction("通道1-通道13", this, SLOT(show_video_13()));
    menu13->addAction("通道4-通道16", this, SLOT(show_video_13()));

    videoMenu->addAction("切换到16画面", this, SLOT(show_video_16()));
    videoMenu->addAction("切换到25画面", this, SLOT(show_video_25()));



}

void VideoPanel::full()
{
    if (actionFull->text() == "切换全屏模式") {
        emit fullScreen(true);
        actionFull->setText("切换正常模式");
    } else {
        emit fullScreen(false);
        actionFull->setText("切换全屏模式");
    }
    //执行全屏处理
}
//删除摄像头
void VideoPanel::deletepoll()
{
    if(m_widget!=nullptr){//先判断是否为空
        m_widget->close();
    }
}


//连接播放一个
void VideoPanel::play_video_all()
{

}

void VideoPanel::snapshot_video_all()
{

}

void VideoPanel::show_video_all()
{
    if (videoType == "1_4") {
        change_video_4(0);
    } else if (videoType == "5_8") {
        change_video_4(4);
    } else if (videoType == "9_12") {
        change_video_4(8);
    } else if (videoType == "13_16") {
        change_video_4(12);
    } else if (videoType == "1_6") {
        change_video_6(0);
    } else if (videoType == "6_11") {
        change_video_6(5);
    } else if (videoType == "11_16") {
        change_video_6(10);
    } else if (videoType == "1_8") {
        change_video_8(0);
    } else if (videoType == "9_16") {
        change_video_8(8);
    } else if (videoType == "1_9") {
        change_video_9(0);
    } else if (videoType == "8_16") {
        change_video_9(7);
    } else if (videoType == "1_13") {
        change_video_13(0);
    } else if (videoType == "4_16") {
        change_video_13(3);
    } else if (videoType == "1_16") {
        change_video_16(0);
    } else if (videoType == "1_25") {
        change_video_25(0);
    } else if (videoType == "1_36") {
        change_video_36(0);
    } else if (videoType == "1_64") {
        change_video_64(0);
    }
}

void VideoPanel::show_video_4()
{
    videoMax = false;
    QString videoType;
    int index = 0;

    QAction *action = (QAction *)sender();
    QString name = action->text();

    if (name == "通道1-通道4") {
        index = 0;
        videoType = "1_4";
    } else if (name == "通道5-通道8") {
        index = 4;
        videoType = "5_8";
    } else if (name == "通道9-通道12") {
        index = 8;
        videoType = "9_12";
    } else if (name == "通道13-通道16") {
        index = 12;
        videoType = "13_16";
    }

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_4(index);
    }
}

void VideoPanel::show_video_6()
{
    videoMax = false;
    QString videoType;
    int index = 0;

    QAction *action = (QAction *)sender();
    QString name = action->text();

    if (name == "通道1-通道6") {
        index = 0;
        videoType = "1_6";
    } else if (name == "通道6-通道11") {
        index = 5;
        videoType = "6_11";
    } else if (name == "通道11-通道16") {
        index = 10;
        videoType = "11_16";
    }

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_6(index);
    }
}

void VideoPanel::show_video_8()
{
    videoMax = false;
    QString videoType;
    int index = 0;

    QAction *action = (QAction *)sender();
    QString name = action->text();

    if (name == "通道1-通道8") {
        index = 0;
        videoType = "1_8";
    } else if (name == "通道9-通道16") {
        index = 8;
        videoType = "9_16";
    }

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_8(index);
    }
}

void VideoPanel::show_video_9()
{
    videoMax = false;
    QString videoType;
    int index = 0;

    QAction *action = (QAction *)sender();
    QString name = action->text();

    if (name == "通道1-通道9") {
        index = 0;
        videoType = "1_9";
    } else if (name == "通道8-通道16") {
        index = 7;
        videoType = "8_16";
    }

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_9(index);
    }
}

void VideoPanel::show_video_13()
{
    QString videoType;
    int index = 0;

    QAction *action = (QAction *)sender();
    QString name = action->text();

    if (name == "通道1-通道13") {
        index = 0;
        videoType = "1_13";
    } else if (name == "通道4-通道16") {
        index = 3;
        videoType = "4_16";
    }

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_13(index);
    }
}

void VideoPanel::show_video_16()
{
    videoMax = false;
    QString videoType;
    int index = 0;
    videoType = "1_16";

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_16(index);
    }
}

void VideoPanel::show_video_25()
{
    videoMax = false;
    QString videoType;
    int index = 0;
    videoType = "1_25";

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_25(index);
    }
}

void VideoPanel::show_video_36()
{
    videoMax = false;
    QString videoType;
    int index = 0;
    videoType = "1_36";

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_36(index);
    }
}

void VideoPanel::show_video_64()
{
    videoMax = false;
    QString videoType;
    int index = 0;
    videoType = "1_64";

    if (this->videoType != videoType) {
        this->videoType = videoType;
        change_video_64(index);
    }
}

void VideoPanel::hide_video_all()
{
    for (int i = 0; i < videoCount; i++) {
        gridLayout->removeWidget(widgets.at(i));
        widgets.at(i)->setVisible(false);
    }
}

void VideoPanel::change_video(int index, int flag)
{
    int count = 0;
    int row = 0;
    int column = 0;

    for (int i = 0; i < videoCount; i++) {
        if (i >= index) {
            gridLayout->addWidget(widgets.at(i), row, column);
            widgets.at(i)->setVisible(true);

            count++;
            column++;
            if (column == flag) {
                row++;
                column = 0;
            }
        }

        if (count == (flag * flag)) {
            break;
        }
    }
}

void VideoPanel::change_video_4(int index)
{
    hide_video_all();
    change_video(index, 2);
}

void VideoPanel::change_video_6(int index)
{
    hide_video_all();
    if (index == 0) {
        gridLayout->addWidget(widgets.at(0), 0, 0, 2, 2);
        gridLayout->addWidget(widgets.at(1), 0, 2, 1, 1);
        gridLayout->addWidget(widgets.at(2), 1, 2, 1, 1);
        gridLayout->addWidget(widgets.at(3), 2, 2, 1, 1);
        gridLayout->addWidget(widgets.at(4), 2, 1, 1, 1);
        gridLayout->addWidget(widgets.at(5), 2, 0, 1, 1);

        for (int i = 0; i < 6; i++) {
            widgets.at(i)->setVisible(true);
        }
    } else if (index == 5) {
        gridLayout->addWidget(widgets.at(5), 0, 0, 2, 2);
        gridLayout->addWidget(widgets.at(6), 0, 2, 1, 1);
        gridLayout->addWidget(widgets.at(7), 1, 2, 1, 1);
        gridLayout->addWidget(widgets.at(8), 2, 2, 1, 1);
        gridLayout->addWidget(widgets.at(9), 2, 1, 1, 1);
        gridLayout->addWidget(widgets.at(10), 2, 0, 1, 1);

        for (int i = 5; i < 11; i++) {
            widgets.at(i)->setVisible(true);
        }
    } else if (index == 10) {
        gridLayout->addWidget(widgets.at(10), 0, 0, 2, 2);
        gridLayout->addWidget(widgets.at(11), 0, 2, 1, 1);
        gridLayout->addWidget(widgets.at(12), 1, 2, 1, 1);
        gridLayout->addWidget(widgets.at(13), 2, 2, 1, 1);
        gridLayout->addWidget(widgets.at(14), 2, 1, 1, 1);
        gridLayout->addWidget(widgets.at(15), 2, 0, 1, 1);

        for (int i = 10; i < 16; i++) {
            widgets.at(i)->setVisible(true);
        }
    }
}

void VideoPanel::change_video_8(int index)
{
    hide_video_all();
    if (index == 0) {
        gridLayout->addWidget(widgets.at(0), 0, 0, 3, 3);
        gridLayout->addWidget(widgets.at(1), 0, 3, 1, 1);
        gridLayout->addWidget(widgets.at(2), 1, 3, 1, 1);
        gridLayout->addWidget(widgets.at(3), 2, 3, 1, 1);
        gridLayout->addWidget(widgets.at(4), 3, 3, 1, 1);
        gridLayout->addWidget(widgets.at(5), 3, 2, 1, 1);
        gridLayout->addWidget(widgets.at(6), 3, 1, 1, 1);
        gridLayout->addWidget(widgets.at(7), 3, 0, 1, 1);

        for (int i = 0; i < 8; i++) {
            widgets.at(i)->setVisible(true);
        }
    } else if (index == 8) {
        gridLayout->addWidget(widgets.at(8), 0, 0, 3, 3);
        gridLayout->addWidget(widgets.at(9), 0, 3, 1, 1);
        gridLayout->addWidget(widgets.at(10), 1, 3, 1, 1);
        gridLayout->addWidget(widgets.at(11), 2, 3, 1, 1);
        gridLayout->addWidget(widgets.at(12), 3, 3, 1, 1);
        gridLayout->addWidget(widgets.at(13), 3, 2, 1, 1);
        gridLayout->addWidget(widgets.at(14), 3, 1, 1, 1);
        gridLayout->addWidget(widgets.at(15), 3, 0, 1, 1);

        for (int i = 8; i < 16; i++) {
            widgets.at(i)->setVisible(true);
        }
    }
}

void VideoPanel::change_video_9(int index)
{
    hide_video_all();
    change_video(index, 3);
}

void VideoPanel::change_video_13(int index)
{
    hide_video_all();
    if (index == 0) {
        gridLayout->addWidget(widgets.at(0), 0, 0, 1, 1);
        gridLayout->addWidget(widgets.at(1), 0, 1, 1, 1);
        gridLayout->addWidget(widgets.at(2), 0, 2, 1, 1);
        gridLayout->addWidget(widgets.at(3), 0, 3, 1, 1);
        gridLayout->addWidget(widgets.at(4), 1, 0, 1, 1);
        gridLayout->addWidget(widgets.at(5), 2, 0, 1, 1);
        gridLayout->addWidget(widgets.at(6), 1, 1, 2, 2);
        gridLayout->addWidget(widgets.at(7), 1, 3, 1, 1);
        gridLayout->addWidget(widgets.at(8), 2, 3, 1, 1);
        gridLayout->addWidget(widgets.at(9), 3, 0, 1, 1);
        gridLayout->addWidget(widgets.at(10), 3, 1, 1, 1);
        gridLayout->addWidget(widgets.at(11), 3, 2, 1, 1);
        gridLayout->addWidget(widgets.at(12), 3, 3, 1, 1);

        for (int i = 0; i < 13; i++) {
            widgets.at(i)->setVisible(true);
        }
    } else if (index == 3) {
        gridLayout->addWidget(widgets.at(3), 0, 0, 1, 1);
        gridLayout->addWidget(widgets.at(4), 0, 1, 1, 1);
        gridLayout->addWidget(widgets.at(5), 0, 2, 1, 1);
        gridLayout->addWidget(widgets.at(6), 0, 3, 1, 1);
        gridLayout->addWidget(widgets.at(7), 1, 0, 1, 1);
        gridLayout->addWidget(widgets.at(8), 2, 0, 1, 1);
        gridLayout->addWidget(widgets.at(9), 1, 1, 2, 2);
        gridLayout->addWidget(widgets.at(10), 1, 3, 1, 1);
        gridLayout->addWidget(widgets.at(11), 2, 3, 1, 1);
        gridLayout->addWidget(widgets.at(12), 3, 0, 1, 1);
        gridLayout->addWidget(widgets.at(13), 3, 1, 1, 1);
        gridLayout->addWidget(widgets.at(14), 3, 2, 1, 1);
        gridLayout->addWidget(widgets.at(15), 3, 3, 1, 1);

        for (int i = 3; i < 16; i++) {
            widgets.at(i)->setVisible(true);
        }
    }
}

void VideoPanel::change_video_16(int index)
{
    hide_video_all();
    change_video(index, 4);
}

void VideoPanel::change_video_25(int index)
{
    hide_video_all();
    change_video(index, 5);
}

void VideoPanel::change_video_36(int index)
{
    hide_video_all();
    change_video(index, 6);
}

void VideoPanel::change_video_64(int index)
{
    hide_video_all();
    change_video(index, 8);
}

void VideoPanel::mtimeout()
{
    for(int i = 0;i < widgets.size();i++){
        if(m_playnum == i){
            widgets.at(i)->open();
        }
    }
    m_playnum++;
    if(m_playnum >= 25){
        m_timer->stop();
    }
}
//摄像头设置
void VideoPanel::openCamera()
{
    if(m_widget!=nullptr){//先判断是否为空
        m_widget->open();
    }
}

void VideoPanel::openCameraAll()
{
    m_timer->start(1000);
}

//摄像头设置
void VideoPanel::setCameraInfo()
{
    if(m_widget!=nullptr){//先判断是否为空x
        CameraSetDialog t_CameraSetDialog;
        connect(&t_CameraSetDialog,SIGNAL(sendCameraInfo(QStringList)),this,SLOT(acceptCameraInfo(QStringList)));
        t_CameraSetDialog.exec();
    }
}

void VideoPanel::closeCameraInfo()
{
    if(m_widget!=nullptr){//先判断是否为空
        m_widget->close();
    }
}

void VideoPanel::cameraPlayAll()
{
    emit sendPlayAllCam();
}
//来自界面
void VideoPanel::acceptCameraInfo(QStringList p_strlist)
{
    qDebug()<<"acceptCameraInfo==="<<p_strlist;
    if(m_widget!=nullptr){//先判断是否为空
        //这里是设备名称，编号，地址
        m_widget->setTitle(p_strlist.at(0));
        QString flagstr =  m_widget->getFlag();

        if (!userDataBasePri::database.open()){//打开数据库
            qCritical() << userDataBasePri::database.lastError().text();
            QMessageBox msgBox;
            msgBox.setText("数据库打开失败.");
            msgBox.exec();
            return;
        }
        QSqlQuery query(userDataBasePri::database);
        query.exec(QString::fromLocal8Bit("update camera_info set url = '%1' where id = '%2'").arg(p_strlist.at(0)).arg(flagstr));

        userDataBasePri::database.close();
    }
}

void VideoPanel::acceptOpenState(bool state)
{
    if(state == true){
        m_playnum++;//添加一个
    }else{

        QMessageBox *box = new QMessageBox(QMessageBox::Information,tr("error"),tr("打开视频地址失败"));
        box->setStyleSheet("QMessageBox { background-color: lightblue; }"\
                           "QMessageBox QLabel { color: darkblue; }"\
                           "QMessageBox QPushButton { background-color: rgb(42,67,101); color: white; }");


        QTimer::singleShot(1500,box,SLOT(accept())); //也可将accept改为close
        box->exec();//box->show();都可以
    }
}
