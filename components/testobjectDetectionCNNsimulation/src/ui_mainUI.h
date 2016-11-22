/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QHBoxLayout *gridLayout;
    QPushButton *pushButtonLeft;
    QPushButton *pushButtonRight;
    QPushButton *pushButtonFront;
    QPushButton *pushButtonBack;
    QPushButton *pushButtonClock;
    QPushButton *pushButtonAnticlock;
    QPushButton *pushButtondetect;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(835, 72);
        gridLayout = new QHBoxLayout(guiDlg);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pushButtonLeft = new QPushButton(guiDlg);
        pushButtonLeft->setObjectName(QString::fromUtf8("pushButtonLeft"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pushButtonLeft->sizePolicy().hasHeightForWidth());
        pushButtonLeft->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtonLeft);

        pushButtonRight = new QPushButton(guiDlg);
        pushButtonRight->setObjectName(QString::fromUtf8("pushButtonRight"));
        sizePolicy.setHeightForWidth(pushButtonRight->sizePolicy().hasHeightForWidth());
        pushButtonRight->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtonRight);

        pushButtonFront = new QPushButton(guiDlg);
        pushButtonFront->setObjectName(QString::fromUtf8("pushButtonFront"));
        sizePolicy.setHeightForWidth(pushButtonFront->sizePolicy().hasHeightForWidth());
        pushButtonFront->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtonFront);

        pushButtonBack = new QPushButton(guiDlg);
        pushButtonBack->setObjectName(QString::fromUtf8("pushButtonBack"));
        sizePolicy.setHeightForWidth(pushButtonBack->sizePolicy().hasHeightForWidth());
        pushButtonBack->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtonBack);

        pushButtonClock = new QPushButton(guiDlg);
        pushButtonClock->setObjectName(QString::fromUtf8("pushButtonClock"));
        sizePolicy.setHeightForWidth(pushButtonClock->sizePolicy().hasHeightForWidth());
        pushButtonClock->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtonClock);

        pushButtonAnticlock = new QPushButton(guiDlg);
        pushButtonAnticlock->setObjectName(QString::fromUtf8("pushButtonAnticlock"));
        sizePolicy.setHeightForWidth(pushButtonAnticlock->sizePolicy().hasHeightForWidth());
        pushButtonAnticlock->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtonAnticlock);

        pushButtondetect = new QPushButton(guiDlg);
        pushButtondetect->setObjectName(QString::fromUtf8("pushButtondetect"));
        sizePolicy.setHeightForWidth(pushButtondetect->sizePolicy().hasHeightForWidth());
        pushButtondetect->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pushButtondetect);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "testODCNNsimulation", 0, QApplication::UnicodeUTF8));
        pushButtonLeft->setText(QApplication::translate("guiDlg", "Move Left", 0, QApplication::UnicodeUTF8));
        pushButtonRight->setText(QApplication::translate("guiDlg", "Move Right", 0, QApplication::UnicodeUTF8));
        pushButtonFront->setText(QApplication::translate("guiDlg", "Move Front", 0, QApplication::UnicodeUTF8));
        pushButtonBack->setText(QApplication::translate("guiDlg", "Move Back", 0, QApplication::UnicodeUTF8));
        pushButtonClock->setText(QApplication::translate("guiDlg", "Rotate Clockwise", 0, QApplication::UnicodeUTF8));
        pushButtonAnticlock->setText(QApplication::translate("guiDlg", "Rotate Anti Clockwise", 0, QApplication::UnicodeUTF8));
        pushButtondetect->setText(QApplication::translate("guiDlg", "Detect Object", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
