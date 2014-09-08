/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "qwt_slider.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *mylabel1;
    QLabel *mylabel2;
    QListWidget *listWidget;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_9;
    QPushButton *pushButton;
    QLabel *label_11;
    QTextEdit *filename;
    QPushButton *save;
    QPushButton *pushButton_2;
    QPushButton *quit;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_8;
    QRadioButton *YUV;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QVBoxLayout *verticalLayout_2;
    QSpinBox *ymin;
    QSpinBox *ymax;
    QSpinBox *umin;
    QSpinBox *umax;
    QSpinBox *vmin;
    QSpinBox *vmax;
    QVBoxLayout *verticalLayout;
    QSlider *yminslider;
    QSlider *ymaxslider;
    QSlider *uminslider;
    QSlider *umaxslider;
    QSlider *vminslider;
    QSlider *vmaxslider;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_7;
    QRadioButton *RGB;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_4;
    QwtSlider *grslidermin;
    QwtSlider *grslidermax;
    QwtSlider *brslidermin;
    QwtSlider *brslidermax;
    QVBoxLayout *verticalLayout_5;
    QDoubleSpinBox *grmin;
    QDoubleSpinBox *grmax;
    QDoubleSpinBox *brmin;
    QDoubleSpinBox *brmax;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_9;
    QLabel *label_7;
    QLabel *label_10;
    QLabel *label_8;
    QWidget *layoutWidget3;
    QVBoxLayout *verticalLayout_12;
    QPushButton *load;
    QVBoxLayout *verticalLayout_11;
    QHBoxLayout *horizontalLayout_3;
    QSpinBox *spinBox_x1;
    QLabel *label_12;
    QHBoxLayout *horizontalLayout_4;
    QSpinBox *spinBox_x2;
    QLabel *label_13;
    QHBoxLayout *horizontalLayout_5;
    QSpinBox *spinBox_y1;
    QLabel *label_14;
    QHBoxLayout *horizontalLayout_6;
    QSpinBox *spinBox_y2;
    QLabel *label_15;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QWidget *layoutWidget4;
    QVBoxLayout *verticalLayout_10;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QMenuBar *menuBar;
    QStatusBar *statusBar;
    QButtonGroup *buttonGroup;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1291, 816);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        mylabel1 = new QLabel(centralWidget);
        mylabel1->setObjectName(QString::fromUtf8("mylabel1"));
        mylabel1->setGeometry(QRect(0, 0, 640, 480));
        mylabel2 = new QLabel(centralWidget);
        mylabel2->setObjectName(QString::fromUtf8("mylabel2"));
        mylabel2->setGeometry(QRect(640, 0, 640, 480));
        listWidget = new QListWidget(centralWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        new QListWidgetItem(listWidget);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        listWidget->setGeometry(QRect(720, 480, 111, 192));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(840, 480, 145, 191));
        verticalLayout_9 = new QVBoxLayout(layoutWidget);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        verticalLayout_9->setContentsMargins(0, 0, 0, 0);
        pushButton = new QPushButton(layoutWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout_9->addWidget(pushButton);

        label_11 = new QLabel(layoutWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        verticalLayout_9->addWidget(label_11);

        filename = new QTextEdit(layoutWidget);
        filename->setObjectName(QString::fromUtf8("filename"));

        verticalLayout_9->addWidget(filename);

        save = new QPushButton(layoutWidget);
        save->setObjectName(QString::fromUtf8("save"));

        verticalLayout_9->addWidget(save);

        pushButton_2 = new QPushButton(layoutWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        verticalLayout_9->addWidget(pushButton_2);

        quit = new QPushButton(layoutWidget);
        quit->setObjectName(QString::fromUtf8("quit"));

        verticalLayout_9->addWidget(quit);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(430, 470, 281, 211));
        verticalLayout_8 = new QVBoxLayout(layoutWidget1);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        YUV = new QRadioButton(layoutWidget1);
        buttonGroup = new QButtonGroup(MainWindow);
        buttonGroup->setObjectName(QString::fromUtf8("buttonGroup"));
        buttonGroup->addButton(YUV);
        YUV->setObjectName(QString::fromUtf8("YUV"));

        verticalLayout_8->addWidget(YUV);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label = new QLabel(layoutWidget1);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_3->addWidget(label);

        label_2 = new QLabel(layoutWidget1);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_3->addWidget(label_2);

        label_3 = new QLabel(layoutWidget1);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_3->addWidget(label_3);

        label_4 = new QLabel(layoutWidget1);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_3->addWidget(label_4);

        label_5 = new QLabel(layoutWidget1);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_3->addWidget(label_5);

        label_6 = new QLabel(layoutWidget1);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_3->addWidget(label_6);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        ymin = new QSpinBox(layoutWidget1);
        ymin->setObjectName(QString::fromUtf8("ymin"));
        ymin->setMaximum(255);

        verticalLayout_2->addWidget(ymin);

        ymax = new QSpinBox(layoutWidget1);
        ymax->setObjectName(QString::fromUtf8("ymax"));
        ymax->setMaximum(255);

        verticalLayout_2->addWidget(ymax);

        umin = new QSpinBox(layoutWidget1);
        umin->setObjectName(QString::fromUtf8("umin"));
        umin->setMaximum(255);

        verticalLayout_2->addWidget(umin);

        umax = new QSpinBox(layoutWidget1);
        umax->setObjectName(QString::fromUtf8("umax"));
        umax->setMaximum(255);

        verticalLayout_2->addWidget(umax);

        vmin = new QSpinBox(layoutWidget1);
        vmin->setObjectName(QString::fromUtf8("vmin"));
        vmin->setMaximum(255);

        verticalLayout_2->addWidget(vmin);

        vmax = new QSpinBox(layoutWidget1);
        vmax->setObjectName(QString::fromUtf8("vmax"));
        vmax->setMaximum(255);

        verticalLayout_2->addWidget(vmax);


        horizontalLayout->addLayout(verticalLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        yminslider = new QSlider(layoutWidget1);
        yminslider->setObjectName(QString::fromUtf8("yminslider"));
        yminslider->setMaximum(255);
        yminslider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(yminslider);

        ymaxslider = new QSlider(layoutWidget1);
        ymaxslider->setObjectName(QString::fromUtf8("ymaxslider"));
        ymaxslider->setMaximum(255);
        ymaxslider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(ymaxslider);

        uminslider = new QSlider(layoutWidget1);
        uminslider->setObjectName(QString::fromUtf8("uminslider"));
        uminslider->setMaximum(255);
        uminslider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(uminslider);

        umaxslider = new QSlider(layoutWidget1);
        umaxslider->setObjectName(QString::fromUtf8("umaxslider"));
        umaxslider->setMaximum(255);
        umaxslider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(umaxslider);

        vminslider = new QSlider(layoutWidget1);
        vminslider->setObjectName(QString::fromUtf8("vminslider"));
        vminslider->setMaximum(255);
        vminslider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(vminslider);

        vmaxslider = new QSlider(layoutWidget1);
        vmaxslider->setObjectName(QString::fromUtf8("vmaxslider"));
        vmaxslider->setMaximum(255);
        vmaxslider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(vmaxslider);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_8->addLayout(horizontalLayout);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(30, 480, 381, 290));
        verticalLayout_7 = new QVBoxLayout(layoutWidget2);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        RGB = new QRadioButton(layoutWidget2);
        buttonGroup->addButton(RGB);
        RGB->setObjectName(QString::fromUtf8("RGB"));

        verticalLayout_7->addWidget(RGB);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        grslidermin = new QwtSlider(layoutWidget2);
        grslidermin->setObjectName(QString::fromUtf8("grslidermin"));
        grslidermin->setProperty("mass", QVariant(0));
        grslidermin->setProperty("borderWidth", QVariant(2));
        grslidermin->setProperty("spacing", QVariant(2));

        verticalLayout_4->addWidget(grslidermin);

        grslidermax = new QwtSlider(layoutWidget2);
        grslidermax->setObjectName(QString::fromUtf8("grslidermax"));

        verticalLayout_4->addWidget(grslidermax);

        brslidermin = new QwtSlider(layoutWidget2);
        brslidermin->setObjectName(QString::fromUtf8("brslidermin"));

        verticalLayout_4->addWidget(brslidermin);

        brslidermax = new QwtSlider(layoutWidget2);
        brslidermax->setObjectName(QString::fromUtf8("brslidermax"));

        verticalLayout_4->addWidget(brslidermax);


        horizontalLayout_2->addLayout(verticalLayout_4);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        grmin = new QDoubleSpinBox(layoutWidget2);
        grmin->setObjectName(QString::fromUtf8("grmin"));
        grmin->setDecimals(3);
        grmin->setMaximum(5);
        grmin->setSingleStep(0.001);

        verticalLayout_5->addWidget(grmin);

        grmax = new QDoubleSpinBox(layoutWidget2);
        grmax->setObjectName(QString::fromUtf8("grmax"));
        grmax->setDecimals(3);
        grmax->setMaximum(5);
        grmax->setSingleStep(0.001);

        verticalLayout_5->addWidget(grmax);

        brmin = new QDoubleSpinBox(layoutWidget2);
        brmin->setObjectName(QString::fromUtf8("brmin"));
        brmin->setDecimals(3);
        brmin->setMaximum(5);
        brmin->setSingleStep(0.001);

        verticalLayout_5->addWidget(brmin);

        brmax = new QDoubleSpinBox(layoutWidget2);
        brmax->setObjectName(QString::fromUtf8("brmax"));
        brmax->setDecimals(3);
        brmax->setMaximum(5);
        brmax->setSingleStep(0.001);

        verticalLayout_5->addWidget(brmax);


        horizontalLayout_2->addLayout(verticalLayout_5);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        label_9 = new QLabel(layoutWidget2);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_6->addWidget(label_9);

        label_7 = new QLabel(layoutWidget2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_6->addWidget(label_7);

        label_10 = new QLabel(layoutWidget2);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        verticalLayout_6->addWidget(label_10);

        label_8 = new QLabel(layoutWidget2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_6->addWidget(label_8);


        horizontalLayout_2->addLayout(verticalLayout_6);


        verticalLayout_7->addLayout(horizontalLayout_2);

        layoutWidget3 = new QWidget(centralWidget);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(990, 480, 101, 265));
        verticalLayout_12 = new QVBoxLayout(layoutWidget3);
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setContentsMargins(11, 11, 11, 11);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        verticalLayout_12->setContentsMargins(0, 0, 0, 0);
        load = new QPushButton(layoutWidget3);
        load->setObjectName(QString::fromUtf8("load"));

        verticalLayout_12->addWidget(load);

        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        spinBox_x1 = new QSpinBox(layoutWidget3);
        spinBox_x1->setObjectName(QString::fromUtf8("spinBox_x1"));
        spinBox_x1->setMaximum(639);

        horizontalLayout_3->addWidget(spinBox_x1);

        label_12 = new QLabel(layoutWidget3);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_3->addWidget(label_12);


        verticalLayout_11->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        spinBox_x2 = new QSpinBox(layoutWidget3);
        spinBox_x2->setObjectName(QString::fromUtf8("spinBox_x2"));
        spinBox_x2->setMaximum(639);

        horizontalLayout_4->addWidget(spinBox_x2);

        label_13 = new QLabel(layoutWidget3);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout_4->addWidget(label_13);


        verticalLayout_11->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        spinBox_y1 = new QSpinBox(layoutWidget3);
        spinBox_y1->setObjectName(QString::fromUtf8("spinBox_y1"));
        spinBox_y1->setMaximum(479);

        horizontalLayout_5->addWidget(spinBox_y1);

        label_14 = new QLabel(layoutWidget3);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_5->addWidget(label_14);


        verticalLayout_11->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        spinBox_y2 = new QSpinBox(layoutWidget3);
        spinBox_y2->setObjectName(QString::fromUtf8("spinBox_y2"));
        spinBox_y2->setMaximum(479);

        horizontalLayout_6->addWidget(spinBox_y2);

        label_15 = new QLabel(layoutWidget3);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        horizontalLayout_6->addWidget(label_15);


        verticalLayout_11->addLayout(horizontalLayout_6);

        pushButton_3 = new QPushButton(layoutWidget3);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        verticalLayout_11->addWidget(pushButton_3);

        pushButton_4 = new QPushButton(layoutWidget3);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        verticalLayout_11->addWidget(pushButton_4);


        verticalLayout_12->addLayout(verticalLayout_11);

        layoutWidget4 = new QWidget(centralWidget);
        layoutWidget4->setObjectName(QString::fromUtf8("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(1100, 480, 100, 70));
        verticalLayout_10 = new QVBoxLayout(layoutWidget4);
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setContentsMargins(11, 11, 11, 11);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        verticalLayout_10->setContentsMargins(0, 0, 0, 0);
        pushButton_5 = new QPushButton(layoutWidget4);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));

        verticalLayout_10->addWidget(pushButton_5);

        pushButton_6 = new QPushButton(layoutWidget4);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));

        verticalLayout_10->addWidget(pushButton_6);

        MainWindow->setCentralWidget(centralWidget);
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        mylabel1->raise();
        mylabel2->raise();
        listWidget->raise();
        label_4->raise();
        label_5->raise();
        label_6->raise();
        vmin->raise();
        vmax->raise();
        vminslider->raise();
        vmaxslider->raise();
        label_4->raise();
        YUV->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1291, 27));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);
        QObject::connect(yminslider, SIGNAL(sliderMoved(int)), ymin, SLOT(setValue(int)));
        QObject::connect(ymin, SIGNAL(valueChanged(int)), yminslider, SLOT(setValue(int)));
        QObject::connect(ymaxslider, SIGNAL(sliderMoved(int)), ymax, SLOT(setValue(int)));
        QObject::connect(ymax, SIGNAL(valueChanged(int)), ymaxslider, SLOT(setValue(int)));
        QObject::connect(uminslider, SIGNAL(sliderMoved(int)), umin, SLOT(setValue(int)));
        QObject::connect(umin, SIGNAL(valueChanged(int)), uminslider, SLOT(setValue(int)));
        QObject::connect(vmin, SIGNAL(valueChanged(int)), vminslider, SLOT(setValue(int)));
        QObject::connect(vminslider, SIGNAL(sliderMoved(int)), vmin, SLOT(setValue(int)));
        QObject::connect(vmaxslider, SIGNAL(sliderMoved(int)), vmax, SLOT(setValue(int)));
        QObject::connect(vmax, SIGNAL(valueChanged(int)), vmaxslider, SLOT(setValue(int)));
        QObject::connect(ymin, SIGNAL(valueChanged(int)), MainWindow, SLOT(ymin()));
        QObject::connect(ymax, SIGNAL(valueChanged(int)), MainWindow, SLOT(ymax()));
        QObject::connect(umin, SIGNAL(valueChanged(int)), MainWindow, SLOT(umin()));
        QObject::connect(vmin, SIGNAL(valueChanged(int)), MainWindow, SLOT(vmin()));
        QObject::connect(vmax, SIGNAL(valueChanged(int)), MainWindow, SLOT(vmax()));
        QObject::connect(YUV, SIGNAL(clicked()), MainWindow, SLOT(yuvclicked()));
        QObject::connect(yminslider, SIGNAL(valueChanged(int)), ymin, SLOT(setValue(int)));
        QObject::connect(ymaxslider, SIGNAL(valueChanged(int)), ymax, SLOT(setValue(int)));
        QObject::connect(uminslider, SIGNAL(valueChanged(int)), umin, SLOT(setValue(int)));
        QObject::connect(vminslider, SIGNAL(valueChanged(int)), vmin, SLOT(setValue(int)));
        QObject::connect(vmaxslider, SIGNAL(valueChanged(int)), vmax, SLOT(setValue(int)));
        QObject::connect(save, SIGNAL(clicked()), MainWindow, SLOT(saveclicked()));
        QObject::connect(load, SIGNAL(clicked()), MainWindow, SLOT(loadclicked()));
        QObject::connect(quit, SIGNAL(clicked()), MainWindow, SLOT(quitclicked()));
        QObject::connect(pushButton, SIGNAL(clicked()), MainWindow, SLOT(asd()));
        QObject::connect(pushButton_2, SIGNAL(clicked()), MainWindow, SLOT(lookupclicked()));
        QObject::connect(umaxslider, SIGNAL(sliderMoved(int)), umax, SLOT(setValue(int)));
        QObject::connect(umax, SIGNAL(valueChanged(int)), umaxslider, SLOT(setValue(int)));
        QObject::connect(umax, SIGNAL(valueChanged(int)), MainWindow, SLOT(umax()));
        QObject::connect(umaxslider, SIGNAL(valueChanged(int)), umax, SLOT(setValue(int)));
        QObject::connect(grslidermax, SIGNAL(sliderMoved(double)), grmax, SLOT(setValue(double)));
        QObject::connect(grmax, SIGNAL(valueChanged(double)), grslidermax, SLOT(setValue(double)));
        QObject::connect(brslidermin, SIGNAL(sliderMoved(double)), brmin, SLOT(setValue(double)));
        QObject::connect(brmin, SIGNAL(valueChanged(double)), brslidermin, SLOT(setValue(double)));
        QObject::connect(grmin, SIGNAL(valueChanged(double)), MainWindow, SLOT(grmin()));
        QObject::connect(grmax, SIGNAL(valueChanged(double)), MainWindow, SLOT(grmax()));
        QObject::connect(brmin, SIGNAL(valueChanged(double)), MainWindow, SLOT(brmin()));
        QObject::connect(brmax, SIGNAL(valueChanged(double)), brslidermax, SLOT(setValue(double)));
        QObject::connect(brslidermax, SIGNAL(sliderMoved(double)), brmax, SLOT(setValue(double)));
        QObject::connect(brmax, SIGNAL(valueChanged(double)), MainWindow, SLOT(brmax()));
        QObject::connect(grmin, SIGNAL(valueChanged(double)), grslidermin, SLOT(setValue(double)));
        QObject::connect(grslidermin, SIGNAL(sliderMoved(double)), grmin, SLOT(setValue(double)));
        QObject::connect(RGB, SIGNAL(clicked()), MainWindow, SLOT(rgbclicked()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        mylabel1->setText(QApplication::translate("MainWindow", "input", 0, QApplication::UnicodeUTF8));
        mylabel2->setText(QApplication::translate("MainWindow", "output", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled = listWidget->isSortingEnabled();
        listWidget->setSortingEnabled(false);
        QListWidgetItem *___qlistwidgetitem = listWidget->item(0);
        ___qlistwidgetitem->setText(QApplication::translate("MainWindow", "Red", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem1 = listWidget->item(1);
        ___qlistwidgetitem1->setText(QApplication::translate("MainWindow", "Yellow", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem2 = listWidget->item(2);
        ___qlistwidgetitem2->setText(QApplication::translate("MainWindow", "Cyan", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem3 = listWidget->item(3);
        ___qlistwidgetitem3->setText(QApplication::translate("MainWindow", "C1", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem4 = listWidget->item(4);
        ___qlistwidgetitem4->setText(QApplication::translate("MainWindow", "C2", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem5 = listWidget->item(5);
        ___qlistwidgetitem5->setText(QApplication::translate("MainWindow", "Black", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem6 = listWidget->item(6);
        ___qlistwidgetitem6->setText(QApplication::translate("MainWindow", "Green", 0, QApplication::UnicodeUTF8));
        QListWidgetItem *___qlistwidgetitem7 = listWidget->item(7);
        ___qlistwidgetitem7->setText(QApplication::translate("MainWindow", "Maroon", 0, QApplication::UnicodeUTF8));
        listWidget->setSortingEnabled(__sortingEnabled);

        pushButton->setText(QApplication::translate("MainWindow", "Start Stream", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "File Name", 0, QApplication::UnicodeUTF8));
        filename->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Cantarell'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'Ubuntu';\"><br /></p></body></html>", 0, QApplication::UnicodeUTF8));
        save->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("MainWindow", "Make lookup table", 0, QApplication::UnicodeUTF8));
        quit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        YUV->setText(QApplication::translate("MainWindow", "YUV", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Ymin", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Ymax", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Umin", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Umax", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Vmin", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Vmax", 0, QApplication::UnicodeUTF8));
        RGB->setText(QApplication::translate("MainWindow", "RGB", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "B/Rmin", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "G/Rmin", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "B/Rmax", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "G/R max", 0, QApplication::UnicodeUTF8));
        load->setText(QApplication::translate("MainWindow", "Load ", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "X1", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "X2", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "Y1", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "Y2", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("MainWindow", "Save lines", 0, QApplication::UnicodeUTF8));
        pushButton_4->setText(QApplication::translate("MainWindow", "Load lines", 0, QApplication::UnicodeUTF8));
        pushButton_5->setText(QApplication::translate("MainWindow", "Save Points", 0, QApplication::UnicodeUTF8));
        pushButton_6->setText(QApplication::translate("MainWindow", "Load Points", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
