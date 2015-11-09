/********************************************************************************
** Form generated from reading UI file 'rsdPlugin.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RSDPLUGIN_H
#define UI_RSDPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout;
    QLabel *label;
    QLabel *label_3;
    QPushButton *_btn5;
    QDoubleSpinBox *_yPosdoubleSpinBox;
    QPushButton *_btn0;
    QDoubleSpinBox *_xPosdoubleSpinBox;
    QFrame *line;
    QLabel *_label_RobotCamImage;
    QLabel *label_4;
    QDoubleSpinBox *_yRotdoubleSpinBox;
    QPushButton *_btn7;
    QLabel *label_2;
    QPushButton *_btn4;
    QPushButton *_btn2;
    QPushButton *_btn3;
    QPushButton *_btn1;
    QPushButton *_btn6;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(468, 479);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(0, 0));
        label->setMaximumSize(QSize(16777215, 20));

        gridLayout->addWidget(label, 5, 0, 1, 1);

        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMinimumSize(QSize(0, 0));
        label_3->setMaximumSize(QSize(16777215, 20));

        gridLayout->addWidget(label_3, 5, 2, 1, 1);

        _btn5 = new QPushButton(dockWidgetContents);
        _btn5->setObjectName(QString::fromUtf8("_btn5"));
        _btn5->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn5, 7, 2, 1, 1);

        _yPosdoubleSpinBox = new QDoubleSpinBox(dockWidgetContents);
        _yPosdoubleSpinBox->setObjectName(QString::fromUtf8("_yPosdoubleSpinBox"));
        _yPosdoubleSpinBox->setMinimumSize(QSize(0, 0));
        _yPosdoubleSpinBox->setDecimals(3);
        _yPosdoubleSpinBox->setMinimum(-0.75);
        _yPosdoubleSpinBox->setMaximum(0.075);
        _yPosdoubleSpinBox->setSingleStep(0.005);

        gridLayout->addWidget(_yPosdoubleSpinBox, 6, 1, 1, 1);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));
        _btn0->setMinimumSize(QSize(0, 100));

        gridLayout->addWidget(_btn0, 1, 0, 1, 4);

        _xPosdoubleSpinBox = new QDoubleSpinBox(dockWidgetContents);
        _xPosdoubleSpinBox->setObjectName(QString::fromUtf8("_xPosdoubleSpinBox"));
        _xPosdoubleSpinBox->setMinimumSize(QSize(0, 0));
        _xPosdoubleSpinBox->setDecimals(3);
        _xPosdoubleSpinBox->setMinimum(-0.1);
        _xPosdoubleSpinBox->setMaximum(0.1);
        _xPosdoubleSpinBox->setSingleStep(0.005);

        gridLayout->addWidget(_xPosdoubleSpinBox, 6, 0, 1, 1);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 4, 0, 1, 4);

        _label_RobotCamImage = new QLabel(dockWidgetContents);
        _label_RobotCamImage->setObjectName(QString::fromUtf8("_label_RobotCamImage"));

        gridLayout->addWidget(_label_RobotCamImage, 0, 0, 1, 4);

        label_4 = new QLabel(dockWidgetContents);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMinimumSize(QSize(0, 0));
        label_4->setMaximumSize(QSize(16777215, 40));
        QFont font;
        font.setPointSize(20);
        font.setBold(true);
        font.setUnderline(true);
        font.setWeight(75);
        label_4->setFont(font);

        gridLayout->addWidget(label_4, 3, 0, 1, 4);

        _yRotdoubleSpinBox = new QDoubleSpinBox(dockWidgetContents);
        _yRotdoubleSpinBox->setObjectName(QString::fromUtf8("_yRotdoubleSpinBox"));
        _yRotdoubleSpinBox->setMinimumSize(QSize(0, 0));
        _yRotdoubleSpinBox->setDecimals(3);
        _yRotdoubleSpinBox->setMinimum(-1.58);
        _yRotdoubleSpinBox->setMaximum(1.58);
        _yRotdoubleSpinBox->setSingleStep(0.01);

        gridLayout->addWidget(_yRotdoubleSpinBox, 6, 2, 1, 1);

        _btn7 = new QPushButton(dockWidgetContents);
        _btn7->setObjectName(QString::fromUtf8("_btn7"));
        _btn7->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn7, 8, 0, 1, 1);

        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMinimumSize(QSize(0, 0));
        label_2->setMaximumSize(QSize(16777215, 20));

        gridLayout->addWidget(label_2, 5, 1, 1, 1);

        _btn4 = new QPushButton(dockWidgetContents);
        _btn4->setObjectName(QString::fromUtf8("_btn4"));
        _btn4->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn4, 7, 0, 1, 1);

        _btn2 = new QPushButton(dockWidgetContents);
        _btn2->setObjectName(QString::fromUtf8("_btn2"));
        _btn2->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn2, 9, 0, 1, 1);

        _btn3 = new QPushButton(dockWidgetContents);
        _btn3->setObjectName(QString::fromUtf8("_btn3"));
        _btn3->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn3, 8, 1, 1, 1);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QString::fromUtf8("_btn1"));
        _btn1->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn1, 7, 1, 1, 1);

        _btn6 = new QPushButton(dockWidgetContents);
        _btn6->setObjectName(QString::fromUtf8("_btn6"));
        _btn6->setMinimumSize(QSize(0, 50));

        gridLayout->addWidget(_btn6, 8, 2, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "rsd plugin", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SamplePlugin", "xPos:", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SamplePlugin", "yRot:", 0, QApplication::UnicodeUTF8));
        _btn5->setText(QApplication::translate("SamplePlugin", "Open gripper", 0, QApplication::UnicodeUTF8));
        _yPosdoubleSpinBox->setSuffix(QApplication::translate("SamplePlugin", " m", 0, QApplication::UnicodeUTF8));
        _btn0->setText(QApplication::translate("SamplePlugin", "Start", 0, QApplication::UnicodeUTF8));
        _xPosdoubleSpinBox->setSuffix(QApplication::translate("SamplePlugin", " m", 0, QApplication::UnicodeUTF8));
        _label_RobotCamImage->setText(QString());
        label_4->setText(QApplication::translate("SamplePlugin", "Test functions", 0, QApplication::UnicodeUTF8));
        _yRotdoubleSpinBox->setSuffix(QApplication::translate("SamplePlugin", " radians", 0, QApplication::UnicodeUTF8));
        _btn7->setText(QApplication::translate("SamplePlugin", "Move back from grasp", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SamplePlugin", "yPos:", 0, QApplication::UnicodeUTF8));
        _btn4->setText(QApplication::translate("SamplePlugin", "Move to grasp pos", 0, QApplication::UnicodeUTF8));
        _btn2->setText(QApplication::translate("SamplePlugin", "Move to drop pos", 0, QApplication::UnicodeUTF8));
        _btn3->setText(QApplication::translate("SamplePlugin", "Move to zero pos", 0, QApplication::UnicodeUTF8));
        _btn1->setText(QApplication::translate("SamplePlugin", "Move to img pos", 0, QApplication::UnicodeUTF8));
        _btn6->setText(QApplication::translate("SamplePlugin", "Close gripper", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RSDPLUGIN_H
