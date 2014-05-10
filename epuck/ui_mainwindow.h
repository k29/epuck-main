/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Sat May 10 05:34:23 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *imgLabel;
    QLabel *fpsLabel;
    QLabel *FPS;
    QLabel *configHeadingLabel;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *botLabel;
    QComboBox *botColor;
    QComboBox *botSerial;
    QHBoxLayout *horizontalLayout_2;
    QLabel *botLabel_2;
    QComboBox *botColor_2;
    QComboBox *botSerial_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *botLabel_3;
    QComboBox *botColor_3;
    QComboBox *botSerial_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *botLabel_4;
    QComboBox *botColor_4;
    QComboBox *botSerial_4;
    QHBoxLayout *horizontalLayout_5;
    QLabel *botLabel_5;
    QComboBox *botColor_5;
    QComboBox *botSerial_5;
    QPushButton *algoStartButton;
    QPushButton *algoStopButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(956, 556);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        imgLabel = new QLabel(centralWidget);
        imgLabel->setObjectName(QString::fromUtf8("imgLabel"));
        imgLabel->setGeometry(QRect(10, 0, 640, 480));
        imgLabel->setFrameShape(QFrame::StyledPanel);
        fpsLabel = new QLabel(centralWidget);
        fpsLabel->setObjectName(QString::fromUtf8("fpsLabel"));
        fpsLabel->setGeometry(QRect(75, 500, 61, 20));
        FPS = new QLabel(centralWidget);
        FPS->setObjectName(QString::fromUtf8("FPS"));
        FPS->setGeometry(QRect(10, 500, 66, 17));
        configHeadingLabel = new QLabel(centralWidget);
        configHeadingLabel->setObjectName(QString::fromUtf8("configHeadingLabel"));
        configHeadingLabel->setGeometry(QRect(680, 20, 191, 20));
        QFont font;
        font.setPointSize(13);
        font.setBold(true);
        font.setWeight(75);
        configHeadingLabel->setFont(font);
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(670, 70, 281, 171));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        botLabel = new QLabel(verticalLayoutWidget);
        botLabel->setObjectName(QString::fromUtf8("botLabel"));

        horizontalLayout->addWidget(botLabel);

        botColor = new QComboBox(verticalLayoutWidget);
        botColor->setObjectName(QString::fromUtf8("botColor"));

        horizontalLayout->addWidget(botColor);

        botSerial = new QComboBox(verticalLayoutWidget);
        botSerial->setObjectName(QString::fromUtf8("botSerial"));

        horizontalLayout->addWidget(botSerial);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        botLabel_2 = new QLabel(verticalLayoutWidget);
        botLabel_2->setObjectName(QString::fromUtf8("botLabel_2"));

        horizontalLayout_2->addWidget(botLabel_2);

        botColor_2 = new QComboBox(verticalLayoutWidget);
        botColor_2->setObjectName(QString::fromUtf8("botColor_2"));

        horizontalLayout_2->addWidget(botColor_2);

        botSerial_2 = new QComboBox(verticalLayoutWidget);
        botSerial_2->setObjectName(QString::fromUtf8("botSerial_2"));

        horizontalLayout_2->addWidget(botSerial_2);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        botLabel_3 = new QLabel(verticalLayoutWidget);
        botLabel_3->setObjectName(QString::fromUtf8("botLabel_3"));

        horizontalLayout_3->addWidget(botLabel_3);

        botColor_3 = new QComboBox(verticalLayoutWidget);
        botColor_3->setObjectName(QString::fromUtf8("botColor_3"));

        horizontalLayout_3->addWidget(botColor_3);

        botSerial_3 = new QComboBox(verticalLayoutWidget);
        botSerial_3->setObjectName(QString::fromUtf8("botSerial_3"));

        horizontalLayout_3->addWidget(botSerial_3);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        botLabel_4 = new QLabel(verticalLayoutWidget);
        botLabel_4->setObjectName(QString::fromUtf8("botLabel_4"));

        horizontalLayout_4->addWidget(botLabel_4);

        botColor_4 = new QComboBox(verticalLayoutWidget);
        botColor_4->setObjectName(QString::fromUtf8("botColor_4"));

        horizontalLayout_4->addWidget(botColor_4);

        botSerial_4 = new QComboBox(verticalLayoutWidget);
        botSerial_4->setObjectName(QString::fromUtf8("botSerial_4"));

        horizontalLayout_4->addWidget(botSerial_4);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        botLabel_5 = new QLabel(verticalLayoutWidget);
        botLabel_5->setObjectName(QString::fromUtf8("botLabel_5"));

        horizontalLayout_5->addWidget(botLabel_5);

        botColor_5 = new QComboBox(verticalLayoutWidget);
        botColor_5->setObjectName(QString::fromUtf8("botColor_5"));

        horizontalLayout_5->addWidget(botColor_5);

        botSerial_5 = new QComboBox(verticalLayoutWidget);
        botSerial_5->setObjectName(QString::fromUtf8("botSerial_5"));

        horizontalLayout_5->addWidget(botSerial_5);


        verticalLayout->addLayout(horizontalLayout_5);

        algoStartButton = new QPushButton(centralWidget);
        algoStartButton->setObjectName(QString::fromUtf8("algoStartButton"));
        algoStartButton->setGeometry(QRect(690, 260, 98, 27));
        algoStopButton = new QPushButton(centralWidget);
        algoStopButton->setObjectName(QString::fromUtf8("algoStopButton"));
        algoStopButton->setGeometry(QRect(830, 260, 98, 27));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 956, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        imgLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        fpsLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        FPS->setText(QApplication::translate("MainWindow", "FPS:", 0, QApplication::UnicodeUTF8));
        configHeadingLabel->setText(QApplication::translate("MainWindow", "epuck configuration", 0, QApplication::UnicodeUTF8));
        botLabel->setText(QApplication::translate("MainWindow", "epuck 1", 0, QApplication::UnicodeUTF8));
        botColor->clear();
        botColor->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "red", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "blue", 0, QApplication::UnicodeUTF8)
        );
        botLabel_2->setText(QApplication::translate("MainWindow", "epuck 2", 0, QApplication::UnicodeUTF8));
        botLabel_3->setText(QApplication::translate("MainWindow", "epuck 3", 0, QApplication::UnicodeUTF8));
        botLabel_4->setText(QApplication::translate("MainWindow", "epuck 4", 0, QApplication::UnicodeUTF8));
        botLabel_5->setText(QApplication::translate("MainWindow", "epuck 5", 0, QApplication::UnicodeUTF8));
        algoStartButton->setText(QApplication::translate("MainWindow", "Start", 0, QApplication::UnicodeUTF8));
        algoStopButton->setText(QApplication::translate("MainWindow", "Pause", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
