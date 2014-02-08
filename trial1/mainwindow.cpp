#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qwt/qwt_slider.h>
#include<stdio.h>
#include<fstream>
#include <QMouseEvent>
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->grslidermin->setRange(0.00,5.00,0.001);
    ui->grslidermax->setRange(0.00,5.00,0.001);
    ui->brslidermin->setRange(0.00,5.00,0.001);
    ui->brslidermax->setRange(0.00,5.00,0.001);
    mine_x = 0;
    bomb_x = 639;
    mine_y = 0;
    bomb_y = 479;
    frame1 = new QImage(640, 480, QImage::Format_ARGB32);
    result1 = new QImage(640, 480, QImage::Format_ARGB32);
    char fl[][25] = {"red", "yellow", "cyan", "c1", "c2", "black", "green", "maroon"};
    for(int i = 0; i < 8; ++i)
        strcpy(fileList[i], fl[i]);
    timer = new QTimer;
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    isCameraOn = 0;
    cam = NULL;
}

void MainWindow::onTimeout()
{
    float y,u,v;
        /* create a window for the video */
        //cvNamedWindow( "Original", CV_WINDOW_AUTOSIZE );

    if(isCameraOn == 1)
    {
            /* get a frame */
            cam->getImage();

            /* always check */

            w=cam->width();
        h=cam->height();
        image2=cvCreateImage(cvSize(w,h),8,1);
        for(int i=0;i<w;i++)
            {
            for(int j=0;j<h;j++)
                {
                    pixelColor3C(cam->rgbimg, i, j, rgb);
                    if(colourscheme==1)
                    {
                        br=(float)rgb[2]/(float)rgb[0];
                        gr=(float)rgb[1]/(float)rgb[0];

                        if(gr>=grminval&&br>=brminval&&gr<=grmaxval&&br<=brmaxval)
                            {
                            returnPixel1C(image2, i, j)=255;
                            }
                        else
                            {
                            returnPixel1C(image2, i, j)=0;
                            }
                    }
                    else
                    {
                        y=0.299*rgb[0]+0.587*rgb[1]+0.114*rgb[2];
                        u=(rgb[0]-y)*0.713+128.0;
                        v=(rgb[2]-y)*0.564+128.0;
                        if(y>=yminval&&y<=ymaxval&&u>=uminval&&u<=umaxval&&v>=vminval&&v<=vmaxval)
                        {
                            returnPixel1C(image2,i,j)=255;

                        }
                        else
                        {
                            returnPixel1C(image2,i,j)=0;
                        }
                    }
                }
            }


        int x1, x2, y1, y2;
        x1 = ui->spinBox_x1->value();
        x2=ui->spinBox_x2->value();
        y1=ui->spinBox_y1->value();
        y2=ui->spinBox_y2->value();
        cvLine(cam->rgbimg, cvPoint(x1,0), cvPoint(x1, 479), cvScalar(127, 0, 0));
        cvLine(cam->rgbimg, cvPoint(x2,0), cvPoint(x2, 479), cvScalar(127, 0, 0));
        cvLine(cam->rgbimg, cvPoint(0, y1), cvPoint(639, y1), cvScalar(127, 0, 0));
        cvLine(cam->rgbimg, cvPoint(0, y2), cvPoint(639, y2), cvScalar(127, 0, 0));
        cvCircle(cam->rgbimg, cvPoint(mine_x, mine_y), 1, cvScalar(0,127,0), 2);
        cvCircle(cam->rgbimg, cvPoint(mine_x, mine_y), 10, cvScalar(0,127,0), 5);
        cvCircle(cam->rgbimg, cvPoint(bomb_x, bomb_y), 10, cvScalar(0,0,127), 5);
        cvCircle(cam->rgbimg, cvPoint(bomb_x, bomb_y), 1, cvScalar(0,0,127), 2);




        IplImage2QImage(cam->rgbimg,frame1 );
        IplImage2QImage(image2, result1);
        cvReleaseImage(&image2);
        //cvShowImage("Result", image);
        //cvShowImage("Result2", image2);

            /* display current frame */
            //cvShowImage( "Original", frame );

            /* exit if user press 'q' */



    ui->mylabel1->setPixmap(QPixmap::fromImage(*frame1));
    ui->mylabel1->show();
    ui->mylabel2->setPixmap(QPixmap::fromImage(*result1));
    ui->mylabel2->show();

    timer->setSingleShot(true);
    timer->start(10);

//    QCoreApplication:: processEvents();
        }

}


void MainWindow::grmax()
{
    grmaxval=ui->grmax->value();

}

void MainWindow::grmin()
{
    grminval=ui->grmin->value();
}

void MainWindow::brmin()
{
    brminval=ui->brmin->value();
}

void MainWindow::brmax()
{
    brmaxval=ui->brmax->value();
}

void MainWindow::ymin()
{
    yminval=ui->ymin->value();

}

void MainWindow::ymax()
{
    ymaxval=ui->ymax->value();
}

void MainWindow::umin()
{
    uminval=ui->umin->value();
}

void MainWindow::umax()
{
    umaxval=ui->umax->value();
}

void MainWindow::vmin()
{
    vminval=ui->vmin->value();
}

void MainWindow::vmax()
{
    vmaxval=ui->vmax->value();
}


void MainWindow::IplImage2QImage(IplImage *iplImg, QImage *image)
{
    int h = iplImg->height;
    int w = iplImg->width;
    int channels = iplImg->nChannels;
    char *data = iplImg->imageData;

    for (int y = 0; y < h; y++, data += iplImg->widthStep)
    {
        for (int x = 0; x < w; x++)
        {
            char r, g, b, a = 0;
            if (channels == 1)
            {
                r = data[x * channels];
                g = data[x * channels];
                b = data[x * channels];
            }
            else if (channels == 3 || channels == 4)
            {
                r = data[x * channels + 2];
                g = data[x * channels + 1];
                b = data[x * channels];
            }

            if (channels == 4)
            {
                a = data[x * channels + 3];
                image->setPixel(x, y, qRgba(r, g, b, a));
            }
            else
            {
                image->setPixel(x, y, qRgb(r, g, b));
            }
        }
    }
}


void MainWindow::asd()
{
    cam = new CamCapture(false);//, "../pic-2.jpg");
    cam->init(1);

    isCameraOn = 1;

    timer->setSingleShot(true);
    timer->start(10);

}



void MainWindow::mousePressEvent(QMouseEvent *event)
{
//    qDebug(event->pos().x());
//    qDebug(event->pos().y());
    if(event->button()==Qt::LeftButton)
    {
        if(event->pos().x() < 640)
            mine_x = event->pos().x();
        if(event->pos().y() < 480)
            mine_y = event->pos().y();
    }
    else
    {
        if(event->pos().x() < 640)
            bomb_x = event->pos().x();
        if(event->pos().y() < 480)
            bomb_y = event->pos().y();
    }
}


void MainWindow::rgbclicked()
{
    colourscheme=1;
}

void MainWindow::yuvclicked()
{
    colourscheme=0;
}

void MainWindow::quitclicked()
{
    name=ui->filename->toPlainText();
    ofstream file;
    file.open(name.toUtf8().constData());
    file.write((char*)&d,sizeof(Data));
    file.close();

}

void MainWindow::loadclicked()
{
    name=ui->filename->toPlainText();
    ifstream file;
    file.open(name.toUtf8().constData());
    file.read((char*)&d, sizeof(Data));
    file.close();
}

void MainWindow::saveclicked()
{
    d.c[x].brminv=brminval;
    d.c[x].brmaxv=brmaxval;
    d.c[x].grminv=grminval;
    d.c[x].grmaxv=grmaxval;
    d.c[x].yminv=yminval;
    d.c[x].ymaxv=ymaxval;
    d.c[x].uminv=uminval;
    d.c[x].umaxv=umaxval;
    d.c[x].vminv=vminval;
    d.c[x].vmaxv=vmaxval;
    d.c[x].coloursch=colourscheme;
}

void MainWindow::lookupclicked()
{
    makeLUT(d, fileList, 8);
}

MainWindow::~MainWindow()
{
//    isCameraOn = 0;
//    delete cam;
    delete ui;
}

void MainWindow::on_listWidget_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
    int row = ui->listWidget->row(current);
    x = row;
    if(d.c[row].coloursch==1)
    {
        ui->RGB->click();
    }
    else
    {
        ui->YUV->click();
    }
    ui->grmin->setValue(d.c[row].grminv);
    ui->grmax->setValue(d.c[row].grmaxv);
    ui->brmin->setValue(d.c[row].brminv);
    ui->brmax->setValue(d.c[row].brmaxv);
    ui->ymin->setValue(d.c[row].yminv);
    ui->ymax->setValue(d.c[row].ymaxv);
    ui->umin->setValue(d.c[row].uminv);
    ui->umax->setValue(d.c[row].umaxv);
    ui->vmin->setValue(d.c[row].vminv);
    ui->vmax->setValue(d.c[row].vmaxv);
}

void MainWindow::on_pushButton_3_clicked()
{
    fstream f1;
    f1.open("../lineCalib.txt", ios::out);
    f1<<ui->spinBox_x1->value()<<endl;
    f1<<ui->spinBox_x2->value()<<endl;
    f1<<ui->spinBox_y1->value()<<endl;
    f1<<ui->spinBox_y2->value()<<endl;
    f1.close();
}

void MainWindow::on_pushButton_4_clicked()
{
    ifstream lineCalib;
    lineCalib.open("../lineCalib.txt");
    int x1, x2, y1, y2;
    lineCalib>>x1;
    lineCalib>>x2;
    lineCalib>>y1;
    lineCalib>>y2;
    lineCalib.close();
    ui->spinBox_x1->setValue(x1);
    ui->spinBox_x2->setValue(x2);
    ui->spinBox_y1->setValue(y1);
    ui->spinBox_y2->setValue(y2);
}

void MainWindow::on_pushButton_5_clicked()
{
    fstream f1;
    f1.open("../pointCalib.txt", ios::out);
    f1<<mine_x<<endl;
    f1<<mine_y<<endl;
    f1<<bomb_x<<endl;
    f1<<bomb_y<<endl;
    f1.close();
}

void MainWindow::on_pushButton_6_clicked()
{
    ifstream pointCalib;
    pointCalib.open("../pointCalib.txt");
    pointCalib>>mine_x;
    pointCalib>>mine_y;
    pointCalib>>bomb_x;
    pointCalib>>bomb_y;
    pointCalib.close();
}
