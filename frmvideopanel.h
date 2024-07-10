#ifndef FRMVIDEOPANEL_H
#define FRMVIDEOPANEL_H

#include <QWidget>

namespace Ui {
class frmVideoPanel;
}

class frmVideoPanel : public QWidget
{
    Q_OBJECT

public:
    explicit frmVideoPanel(QWidget *parent = 0);
    ~frmVideoPanel();


private slots:
    void mtimeout();

private:
    Ui::frmVideoPanel *ui;
    QTimer *m_timer;
    int     m_playnum;
};

#endif // FRMVIDEOPANEL_H
