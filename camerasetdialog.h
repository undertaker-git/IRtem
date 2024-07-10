#ifndef CAMERASETDIALOG_H
#define CAMERASETDIALOG_H

#include <QDialog>
#include <QStringList>
namespace Ui {
class CameraSetDialog;
}

class CameraSetDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraSetDialog(QWidget *parent = nullptr);
    ~CameraSetDialog();
public:
  virtual void showEvent(QShowEvent *event);

signals:
    void sendCameraInfo(QStringList);

private slots:
    void on_pushButton_a_clicked();

    void on_pushButton_b_clicked();

    void on_comboBox_currentTextChanged(const QString &arg1);

private:
    Ui::CameraSetDialog *ui;


};

#endif // CAMERASETDIALOG_H
