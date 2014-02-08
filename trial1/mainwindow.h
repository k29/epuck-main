#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <opencv2/opencv.hpp>
#include <imgproc.h>
#include <camcapture.h>
#include <QListWidget>
#include <QTimer>

namespace Ui {
class MainWindow;
}

struct ColourData
{
    float grminv, grmaxv, brminv, brmaxv;
    int yminv, ymaxv, uminv, umaxv, vminv, vmaxv,coloursch;
};

struct Data
{
    ColourData c[8];
};

void makeLUT(Data &d, char fileList[][25], int nColors);

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    float grminval, grmaxval, brminval, brmaxval;
    int yminval, ymaxval, uminval, umaxval, vminval, vmaxval,colourscheme;


private:
    Ui::MainWindow *ui;
    IplImage *frame;
    IplImage *image2;
    QImage *frame1;
    QImage *result1;
    QString name;
    QString text;
    int key;
    int h,w;
    int rgb[3];
    float br;
    float gr;
    int x;
    int mine_x, mine_y, bomb_x, bomb_y;
    Data d;
    char fileList[8][25];
    CamCapture* cam;
    int isCameraOn;
    QTimer* timer;

public slots:
    void grmax();
    void grmin();
    void brmin();
    void brmax();
    void ymin();
    void ymax();
    void umin();
    void umax();
    void vmin();
    void vmax();
    void rgbclicked();
    void yuvclicked();
    void IplImage2QImage(IplImage *iplImg, QImage *image);
    void saveclicked();
    void loadclicked();
    void quitclicked();
    void lookupclicked();
    void asd();
    void mousePressEvent(QMouseEvent* event);

    //void new_func1();

private slots:
    void on_listWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void onTimeout();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
};

#endif // MAINWINDOW_H
