/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.11.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "MyLabel.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *PhotoCapture;
    QPushButton *FFCButton;
    QGraphicsView *graphicsView;
    MyLabel *myLabel_Ir;
    QGraphicsView *graphicsView_RGB;
    MyLabel *myLabel_Rgb;
    QPushButton *switchModel;
    QPushButton *RGBcapture;
    QLabel *temperatureBar;
    QLabel *minTemperature_all;
    QLabel *maxTemperature_all;
    QPushButton *avgTemperature;
    QPushButton *maxTemperature;
    QPushButton *minTemperature;
    QCheckBox *checkBox;
    QPushButton *Send_pushButton;
    QPushButton *RgbAndIRCapture;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(786, 463);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        PhotoCapture = new QPushButton(centralWidget);
        PhotoCapture->setObjectName(QStringLiteral("PhotoCapture"));
        PhotoCapture->setGeometry(QRect(240, 330, 61, 31));
        FFCButton = new QPushButton(centralWidget);
        FFCButton->setObjectName(QStringLiteral("FFCButton"));
        FFCButton->setGeometry(QRect(160, 330, 51, 31));
        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(15, 1, 401, 321));
        myLabel_Ir = new MyLabel(centralWidget);
        myLabel_Ir->setObjectName(QStringLiteral("myLabel_Ir"));
        myLabel_Ir->setGeometry(QRect(20, 10, 391, 311));
        graphicsView_RGB = new QGraphicsView(centralWidget);
        graphicsView_RGB->setObjectName(QStringLiteral("graphicsView_RGB"));
        graphicsView_RGB->setGeometry(QRect(420, 0, 361, 321));
        myLabel_Rgb = new MyLabel(centralWidget);
        myLabel_Rgb->setObjectName(QStringLiteral("myLabel_Rgb"));
        myLabel_Rgb->setGeometry(QRect(430, 20, 341, 291));
        switchModel = new QPushButton(centralWidget);
        switchModel->setObjectName(QStringLiteral("switchModel"));
        switchModel->setGeometry(QRect(330, 330, 61, 30));
        switchModel->setCheckable(true);
        RGBcapture = new QPushButton(centralWidget);
        RGBcapture->setObjectName(QStringLiteral("RGBcapture"));
        RGBcapture->setGeometry(QRect(610, 330, 71, 30));
        temperatureBar = new QLabel(centralWidget);
        temperatureBar->setObjectName(QStringLiteral("temperatureBar"));
        temperatureBar->setGeometry(QRect(70, 370, 331, 20));
        minTemperature_all = new QLabel(centralWidget);
        minTemperature_all->setObjectName(QStringLiteral("minTemperature_all"));
        minTemperature_all->setGeometry(QRect(10, 370, 70, 20));
        maxTemperature_all = new QLabel(centralWidget);
        maxTemperature_all->setObjectName(QStringLiteral("maxTemperature_all"));
        maxTemperature_all->setGeometry(QRect(420, 370, 71, 20));
        avgTemperature = new QPushButton(centralWidget);
        avgTemperature->setObjectName(QStringLiteral("avgTemperature"));
        avgTemperature->setGeometry(QRect(430, 330, 41, 30));
        maxTemperature = new QPushButton(centralWidget);
        maxTemperature->setObjectName(QStringLiteral("maxTemperature"));
        maxTemperature->setGeometry(QRect(490, 330, 41, 30));
        minTemperature = new QPushButton(centralWidget);
        minTemperature->setObjectName(QStringLiteral("minTemperature"));
        minTemperature->setGeometry(QRect(550, 330, 41, 30));
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setGeometry(QRect(20, 330, 101, 27));
        Send_pushButton = new QPushButton(centralWidget);
        Send_pushButton->setObjectName(QStringLiteral("Send_pushButton"));
        Send_pushButton->setGeometry(QRect(700, 330, 71, 30));
        RgbAndIRCapture = new QPushButton(centralWidget);
        RgbAndIRCapture->setObjectName(QStringLiteral("RgbAndIRCapture"));
        RgbAndIRCapture->setGeometry(QRect(610, 370, 161, 30));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 786, 28));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        PhotoCapture->setText(QApplication::translate("MainWindow", "CapIR", nullptr));
        FFCButton->setText(QApplication::translate("MainWindow", "FFC", nullptr));
        myLabel_Ir->setText(QApplication::translate("MainWindow", "                                                         NJU", nullptr));
        myLabel_Rgb->setText(QApplication::translate("MainWindow", "                                                        RGB", nullptr));
        switchModel->setText(QApplication::translate("MainWindow", "Switch", nullptr));
        RGBcapture->setText(QApplication::translate("MainWindow", "CapRGB", nullptr));
        temperatureBar->setText(QApplication::translate("MainWindow", "             TextLabel", nullptr));
        minTemperature_all->setText(QApplication::translate("MainWindow", "TextLabel", nullptr));
        maxTemperature_all->setText(QApplication::translate("MainWindow", "TextLabel", nullptr));
        avgTemperature->setText(QApplication::translate("MainWindow", "avg", nullptr));
        maxTemperature->setText(QApplication::translate("MainWindow", "max", nullptr));
        minTemperature->setText(QApplication::translate("MainWindow", "min", nullptr));
        checkBox->setText(QApplication::translate("MainWindow", "detect", nullptr));
        Send_pushButton->setText(QApplication::translate("MainWindow", "Send", nullptr));
        RgbAndIRCapture->setText(QApplication::translate("MainWindow", "Capture IR and RGB", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
