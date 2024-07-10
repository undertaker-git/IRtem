#ifndef MYLABEL_H
#define MYLABEL_H

#include <QtCore>
#include <QWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>


//we extend QLabel to give it an extra slot, setImage
//this is because we can't pass a QPixmap from our thread
//so we have to pass a QImage and turn the QImage into a QPixmap on our end
class MyLabel : public QLabel {
  Q_OBJECT

  public:
    MyLabel(QWidget *parent = 0);
    ~MyLabel();

protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
    void RightMousePressEvent(QMouseEvent *event);
    void MultipaintEvent(QPaintEvent *event);
    void SinglepaintEvent(QPaintEvent *event);
    void CrossCursorPaintevent(int mousepointX, int mousepointY, QPaintEvent *event);
    void displayTemperatureEvent(int mousepointX, int mousepointY, float displayTemperature,QPaintEvent *event);
    void drawRectangleEvent(int rectX, int rectY, int rectWidth, int rectHeight , float rectTemperature ,QPaintEvent *event);

  public slots:
    void setImage(QImage);

private:
    QPoint mousePoint;
    bool isMousePress = false;
};
struct candidate
{
    int CandidateNumber;
    int candidateXStart[5];
    int candidateYStart[5];
    int candidateXEnd[5];
    int candidateYEnd[5];
    int candidateW[5];
    int candidateH[5];
    float candidateTemperature[5];
    int MousePointX;
    int MousePointY;
};

void CandidateInit();
#endif
