#pragma execution_character_set("utf-8")
#include "camerasetdialog.h"
#include "ui_camerasetdialog.h"
#include <QDir>
#include <QApplication>
#include <QIntValidator>
#include <QDateTime>
#include <QMessageBox>
#include <QCamera>
#include <QCameraInfo>

CameraSetDialog::CameraSetDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraSetDialog)
{
    ui->setupUi(this);
    setWindowTitle("网络摄像头设置");
}

CameraSetDialog::~CameraSetDialog()
{
    delete ui;
}

void CameraSetDialog::showEvent(QShowEvent *event)
{
    // 获取所有可用摄像头信息列表
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    ui->comboBox->clear();
    // 遍历摄像头列表
    for (const QCameraInfo& camera : cameras) {
        qDebug() << "Camera Name:" << camera.deviceName();
        //        qDebug() << "Camera Description:" << camera.description();
        //     qDebug() << "Camera Orientation:" << camera.orientation();
        //       qDebug() << "Camera Position:" << camera.position();
        QStringList t_list = camera.deviceName().split("&");
        ui->comboBox->addItem(camera.deviceName());
    }
}

void CameraSetDialog::on_pushButton_a_clicked()
{
    if(ui->lineEdit_url->text().size() < 2){
        QMessageBox msgBox;
        msgBox.setText("请正确输入摄像头名称和地址.");
        msgBox.exec();
        return;
    }

    QStringList t_strlist;
    t_strlist.append(ui->lineEdit_url->text());
    emit sendCameraInfo(t_strlist);
    done(1);
}

void CameraSetDialog::on_pushButton_b_clicked()
{
    done(0);
}

void CameraSetDialog::on_comboBox_currentTextChanged(const QString &arg1)
{
    ui->lineEdit_url->setText(arg1);
}
