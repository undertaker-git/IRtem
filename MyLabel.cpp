#include "MyLabel.h"
#include "LeptonThread.h"
#include <QDebug>

QPoint lastPoint;
QPoint endPoint;
extern int changeBox_flag;
extern float mouseTemperature;

candidate Candidate;

MyLabel::MyLabel(QWidget *parent) : QLabel(parent)
{
    CandidateInit();
}
MyLabel::~MyLabel()
{
}

void CandidateInit()
{
    Candidate.CandidateNumber = 0;
    for(int i = 0;i<5;i++)
    {
        Candidate.candidateXStart[i] = 0;
        Candidate.candidateYStart[i] = 0;
        Candidate.candidateXEnd[i] = 0;
        Candidate.candidateYEnd[i] = 0;
        Candidate.candidateW[i] = 0;
        Candidate.candidateH[i] = 0;
        Candidate.candidateTemperature[i] = 37.5;
    }
    qDebug()<<"CandidateInit finish";
}

//when the system calls setImage, we'll set the label's pixmap
void MyLabel::setImage(QImage image) {
  QPixmap pixmap = QPixmap::fromImage(image);
  int w = this->width();
  int h = this->height();
  setPixmap(pixmap.scaled(w, h, Qt::KeepAspectRatio));
}

void MyLabel::paintEvent(QPaintEvent *event)
{
    MultipaintEvent(event);
}

void MyLabel::MultipaintEvent(QPaintEvent *event)
{
    QLabel::paintEvent(event);
    CrossCursorPaintevent(Candidate.MousePointX, Candidate.MousePointY, event);
    if(changeBox_flag)
    {
        for(int i = 0; i<Candidate.CandidateNumber; i++)
        {
            drawRectangleEvent(Candidate.candidateXStart[i], Candidate.candidateYStart[i], Candidate.candidateW[i], Candidate.candidateH[i] , Candidate.candidateTemperature[i], event);
        }
    }
}

void MyLabel::SinglepaintEvent(QPaintEvent *event)
{
    int x, y, w, h;
    x = lastPoint.x();
    y = lastPoint.y();
    w = endPoint.x() - x;
    h = endPoint.y() - y;
    QLabel::paintEvent(event);

    if(changeBox_flag)
    {
        QPainter painter2(this);
        this->setMouseTracking(true);
        painter2.setPen(QPen(Qt::yellow,3));
        painter2.drawLine(mousePoint.x()-4,mousePoint.y(),mousePoint.x()+4,mousePoint.y());
        painter2.drawLine(mousePoint.x(),mousePoint.y()-4,mousePoint.x(),mousePoint.y()+4);
        if((endPoint.x()>0)&&(endPoint.x()<480)&&(endPoint.y()>0)&&(endPoint.y()<360)&&w&&h)
        {
            QPainter painter1(this);
            this->setMouseTracking(false);
            painter1.setPen(QPen(Qt::red,4));
            painter1.drawRect(QRect(x,y,w,h));
        }
    }
    //QString str = QString::number(num, 'f', 2);
    //mouseTemperature = mouseTemperature;
    QString str = "("+QString::number(float(mouseTemperature), 'f', 1)+")";
    QPainterPath path;
    QPainter painter3(this);

    QFont font("serif", 20);//设置字体样式
    font.setBold(true);//设置粗体

    if(mouseTemperature>0&&lastPoint!=endPoint)
    {
        path.addText(lastPoint.x(), lastPoint.y(), font, str);
        painter3.fillPath(path, QBrush(Qt::yellow));//字体颜色
        painter3.drawPath(path);
    }

}

void MyLabel::CrossCursorPaintevent(int mousepointX, int mousepointY, QPaintEvent *event)
{
    QPainter painter2(this);
    this->setMouseTracking(true);
    painter2.setPen(QPen(Qt::yellow,3));
    painter2.drawLine(mousepointX-4,mousepointY,mousepointX+4,mousepointY);
    painter2.drawLine(mousepointX,mousepointY-4,mousepointX,mousepointY+4);
}

void MyLabel::displayTemperatureEvent(int mousepointX, int mousepointY, float displayTemperature,QPaintEvent *event)
{
    QString str = "("+QString::number(displayTemperature, 'f', 1)+")";
    QPainterPath path;
    QPainter painter3(this);

    QFont font("serif", 20);//设置字体样式
    font.setBold(true);//设置粗体

    if(displayTemperature>0)
    {
        path.addText(mousepointX, mousepointY, font, str);
        painter3.fillPath(path, QBrush(Qt::yellow));//字体颜色
        painter3.drawPath(path);
    }
}

void MyLabel::drawRectangleEvent(int rectX, int rectY, int rectWidth, int rectHeight , float rectTemperature,QPaintEvent *event)
{
    if((rectX>0)&&(rectX<480)&&(rectY>0)&&(rectY<360)&&(rectWidth>0)&&(rectHeight>0))
    {
        QPainter painter1(this);
        this->setMouseTracking(false);
        painter1.setPen(QPen(Qt::red,4));
        painter1.drawRect(QRect(rectX,rectY,rectWidth,rectHeight));
    }
    //qDebug()<<"w = " << rectWidth << "  h = " << rectHeight;
    displayTemperatureEvent(rectX, rectY, rectTemperature, event);
}

void MyLabel::mousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        lastPoint = event->pos();
        if(Candidate.CandidateNumber>4)
        {
            Candidate.CandidateNumber = 0;
        }
        Candidate.candidateXStart[Candidate.CandidateNumber] = event->pos().x();
        Candidate.candidateYStart[Candidate.CandidateNumber] = event->pos().y();
        endPoint = lastPoint;
        isMousePress = true;
        //Candidate.CandidateNumber++;
        qDebug()<<"XStart= "<<Candidate.candidateXStart[Candidate.CandidateNumber]<< "  YStart= "<<Candidate.candidateYStart[Candidate.CandidateNumber];
    }
}

void MyLabel::mouseMoveEvent(QMouseEvent *event)
{
    if(isMousePress==true)
    {
        endPoint = event->pos();
        Candidate.candidateW[Candidate.CandidateNumber] = event->pos().x() - Candidate.candidateXStart[Candidate.CandidateNumber];
        Candidate.candidateH[Candidate.CandidateNumber] = event->pos().y() - Candidate.candidateYStart[Candidate.CandidateNumber];
        update();
    }
    mousePoint = event->pos();
    Candidate.MousePointX = event->pos().x();
    Candidate.MousePointY = event->pos().y();
    update();
}

void MyLabel::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        endPoint = event->pos();
        Candidate.candidateXEnd[Candidate.CandidateNumber] = event->pos().x();
        Candidate.candidateYEnd[Candidate.CandidateNumber] = event->pos().y();
        update();
        isMousePress = false;
    }
    mousePoint = event->pos();
    update();
    qDebug()<<"XEnd= "<<Candidate.candidateXEnd[Candidate.CandidateNumber]<< "  YEnd= "<<Candidate.candidateYEnd[Candidate.CandidateNumber];
    Candidate.CandidateNumber++;
    RightMousePressEvent(event);
}

void MyLabel::RightMousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::RightButton)
    {
        Candidate.CandidateNumber = 0;
        CandidateInit();
    }
}
