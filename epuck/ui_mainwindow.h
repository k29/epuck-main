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
#include <QtGui/QComboBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
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
    QPushButton *algoStartButton;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_3;
    QGridLayout *gridLayout;
    QLabel *Algorithm;
    QLabel *Activation;
    QComboBox *algoComboBox;
    QComboBox *algoActivationComboBox;
    QPushButton *algoStopButton;
    QLabel *label_4;
    QPlainTextEdit *resultText;
    QPushButton *resetButton;
    QSpinBox *configSpinBox;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(988, 557);
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
        algoStartButton = new QPushButton(centralWidget);
        algoStartButton->setObjectName(QString::fromUtf8("algoStartButton"));
        algoStartButton->setEnabled(false);
        algoStartButton->setGeometry(QRect(760, 420, 98, 27));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(660, 50, 314, 267));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(verticalLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_2->addWidget(label_3);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        Algorithm = new QLabel(verticalLayoutWidget);
        Algorithm->setObjectName(QString::fromUtf8("Algorithm"));

        gridLayout->addWidget(Algorithm, 1, 0, 1, 1);

        Activation = new QLabel(verticalLayoutWidget);
        Activation->setObjectName(QString::fromUtf8("Activation"));

        gridLayout->addWidget(Activation, 2, 0, 1, 1);

        algoComboBox = new QComboBox(verticalLayoutWidget);
        algoComboBox->setObjectName(QString::fromUtf8("algoComboBox"));

        gridLayout->addWidget(algoComboBox, 1, 1, 1, 1);

        algoActivationComboBox = new QComboBox(verticalLayoutWidget);
        algoActivationComboBox->setObjectName(QString::fromUtf8("algoActivationComboBox"));

        gridLayout->addWidget(algoActivationComboBox, 2, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        algoStopButton = new QPushButton(verticalLayoutWidget);
        algoStopButton->setObjectName(QString::fromUtf8("algoStopButton"));

        verticalLayout_2->addWidget(algoStopButton);

        label_4 = new QLabel(verticalLayoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_2->addWidget(label_4);

        resultText = new QPlainTextEdit(verticalLayoutWidget);
        resultText->setObjectName(QString::fromUtf8("resultText"));

        verticalLayout_2->addWidget(resultText);

        resetButton = new QPushButton(centralWidget);
        resetButton->setObjectName(QString::fromUtf8("resetButton"));
        resetButton->setGeometry(QRect(760, 360, 98, 31));
        configSpinBox = new QSpinBox(centralWidget);
        configSpinBox->setObjectName(QString::fromUtf8("configSpinBox"));
        configSpinBox->setGeometry(QRect(690, 360, 51, 31));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 988, 29));
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
        algoStartButton->setText(QString());
        label_3->setText(QApplication::translate("MainWindow", "Options", 0, QApplication::UnicodeUTF8));
        Algorithm->setText(QApplication::translate("MainWindow", "Algorithm", 0, QApplication::UnicodeUTF8));
        Activation->setText(QApplication::translate("MainWindow", "Activation", 0, QApplication::UnicodeUTF8));
        algoComboBox->clear();
        algoComboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Mid - Point", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "State based", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Lyndon words", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Node counting", 0, QApplication::UnicodeUTF8)
        );
        algoActivationComboBox->clear();
        algoActivationComboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "One at a time (prob 0)", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "0.5", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "0.25", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Round robin (dining philosopher)", 0, QApplication::UnicodeUTF8)
        );
        algoStopButton->setText(QApplication::translate("MainWindow", "Start/Pause", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Result", 0, QApplication::UnicodeUTF8));
        resetButton->setText(QApplication::translate("MainWindow", "Reset", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
