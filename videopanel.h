#ifndef VIDEOPANEL_H
#define VIDEOPANEL_H

/**
 * 视频监控画面控件
 * 1:目前颜色都写死在样式表,可自行更改或者拓展属性设置
 */
#include <QWidget>
#include <QDebug>
#include <QThread>
#include <QStringList>
#include <QLabel>
#include <QVector>

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <QDir>
#include <QDirIterator>
#include "qcamerauiwidget.h"
#include "userdatabase.h"

using namespace cv;
using namespace std;

class QMenu;
class QLabel;
class QGridLayout;


#ifdef quc
#if (QT_VERSION < QT_VERSION_CHECK(5,7,0))
#include <QtDesigner/QDesignerExportWidget>
#else
#include <QtUiPlugin/QDesignerExportWidget>
#endif

class QDESIGNER_WIDGET_EXPORT VideoPanel : public QWidget
        #else

namespace Ui {
class VideoPanel;
}
class VideoPanel : public QWidget
        #endif
{
    Q_OBJECT

public:
    explicit VideoPanel(QWidget *parent = 0);
    ~VideoPanel();


    void closeAllVideo();
protected:
    virtual bool eventFilter(QObject *watched, QEvent *event);
    virtual void showEvent(QShowEvent *event);

public slots:
    void acceptCameraInfo(QStringList);

    void acceptOpenState(bool);

    void ReadFrame_RGB();
    void NetReadFarme();
    void RGBcapture();
    void NetImage_160_IR(QImage Image_160);
    void NetSend_264buf(uint8_t * outbuf,uint16_t outbuf_size);

    void on_RgbAndIRCapture_released();


private:
    QGridLayout   *gridLayout;    //表格布局存放视频标签
    bool           videoMax;      //是否最大化
    int            videoCount;    //视频通道个数
    QString        videoType;     //当前画面类型
    QMenu         *videoMenu;     //右键菜单
    QAction       *actionFull;    //全屏动作
    QAction       *actionDel;    //删除作
    QAction       *actionSave;    //保存
    QAction       *actionSet;     //设置动作
    QAction       *actionOpen;     //打开动作
    QAction       *actionClose;     //设置动作

    QAction       *actionOpenAll;    //
    QList<QCameraUiWidget*> widgets;   //视频标签集合

    Ui::VideoPanel *ui;
    QImage QImage_rgb;
    CvCapture *Camera_rgb;
    IplImage *Frame_rgb;
    QImage myBar;
    QRgb barColor;
    int frame_flag = 0;
    int frame_num = 0;

    QTimer    *rgbtimer;
    QImage    *imag;
    QImage    *ircimag;
    VideoCapture capture;
    Mat matimage;
    Mat ircmat;
    Mat imggray;

    int count=0;

protected :
    void RGBacqusition(Mat image_in);
    void RgbAndIrAcquisition_RGB(Mat image_in);
    void temperatureBar();
    void OpenCamara_Test();
    void NetSend_ReadFarme();
    int   SystemShutdown_Detect(void);
    void  SystemShutdown(void);
    Mat QImage2cvMat(QImage image);
    QImage cvMat2QImage(const Mat& mat);

public:
    QSize sizeHint()            const;
    QSize minimumSizeHint()     const;
signals:
    void sendDisWidget(QString );
    void sendPlayBackCam(QString ,QString,QString);
    void sendPlayAllCam();

private slots:
    void initControl();  //初始化控制
    void initForm();     //初始化界面
    void initMenu();     //初始化菜单
    void full();         //切换全屏模式
    void deletepoll();         //删除

    void openCamera();         //打开
    void openCameraAll();         //打开
    void setCameraInfo();//摄像头设置
    void closeCameraInfo();//摄像头设置

    void cameraPlayAll();//全部播放
public slots:
    void play_video_all();//播放全部
    void snapshot_video_all();//截图所有视频

    void show_video_all();//显示全部
    void show_video_4();  //显示4个通道视频
    void show_video_6();  //显示6个通道视频
    void show_video_8();  //显示8个通道视频
    void show_video_9();  //显示9个通道视频
    void show_video_13(); //显示13个通道视频
    void show_video_16(); //显示16个通道视频
    void show_video_25(); //显示25个通道视频
    void show_video_36(); //显示36个通道视频
    void show_video_64(); //显示64个通道视频

    void hide_video_all();//隐藏全部
    void change_video(int index, int flag);//
    void change_video_4(int index);//显示4个通道从某个通道开始
    void change_video_6(int index);//显示6个通道
    void change_video_8(int index);//
    void change_video_9(int index);//
    void change_video_13(int index);//
    void change_video_16(int index);//
    void change_video_25(int index);//
    void change_video_36(int index);//
    void change_video_64(int index);//


private slots:
    void mtimeout();


signals:
    void fullScreen(bool full);//是否全屏

private:
    QCameraUiWidget *m_widget;
    QTimer *m_timer;
    int     m_playnum;

public:

    QVector<QVector<QString>> camera_info;
};

#endif // VIDEOPANEL_H
