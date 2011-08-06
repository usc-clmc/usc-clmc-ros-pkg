/********************************************************************************
** Form generated from reading UI file 'pr2_dynamic_movement_primitive_gui_main_window.ui'
**
** Created: Fri Aug 5 17:03:38 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef PR2_DYNAMIC_MOVEMENT_PRIMITIVE_GUI_MAIN_WINDOW_H
#define PR2_DYNAMIC_MOVEMENT_PRIMITIVE_GUI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_pr2_dynamic_movement_primitive_gui
{
public:
    QFrame *frame;
    QListWidget *running_controller_list_;
    QLabel *running_controller_label;
    QPushButton *execute_button_;
    QPushButton *update_button_;
    QListWidget *stopped_controller_list_;
    QLabel *stopped_controller_label;
    QPushButton *start_button_;
    QPushButton *stop_button_;
    QPushButton *reload_button_;

    void setupUi(QWidget *pr2_dynamic_movement_primitive_gui)
    {
        if (pr2_dynamic_movement_primitive_gui->objectName().isEmpty())
            pr2_dynamic_movement_primitive_gui->setObjectName(QString::fromUtf8("pr2_dynamic_movement_primitive_gui"));
        pr2_dynamic_movement_primitive_gui->setWindowModality(Qt::NonModal);
        pr2_dynamic_movement_primitive_gui->resize(792, 530);
        frame = new QFrame(pr2_dynamic_movement_primitive_gui);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(6, 6, 779, 519));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        running_controller_list_ = new QListWidget(frame);
        running_controller_list_->setObjectName(QString::fromUtf8("running_controller_list_"));
        running_controller_list_->setGeometry(QRect(8, 20, 330, 489));
        QFont font;
        font.setFamily(QString::fromUtf8("FreeSans"));
        font.setPointSize(9);
        running_controller_list_->setFont(font);
        running_controller_label = new QLabel(frame);
        running_controller_label->setObjectName(QString::fromUtf8("running_controller_label"));
        running_controller_label->setGeometry(QRect(10, 4, 227, 16));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        running_controller_label->setFont(font1);
        execute_button_ = new QPushButton(frame);
        execute_button_->setObjectName(QString::fromUtf8("execute_button_"));
        execute_button_->setEnabled(false);
        execute_button_->setGeometry(QRect(344, 150, 90, 27));
        execute_button_->setFont(font);
        update_button_ = new QPushButton(frame);
        update_button_->setObjectName(QString::fromUtf8("update_button_"));
        update_button_->setEnabled(true);
        update_button_->setGeometry(QRect(344, 22, 90, 27));
        update_button_->setFont(font);
        stopped_controller_list_ = new QListWidget(frame);
        stopped_controller_list_->setObjectName(QString::fromUtf8("stopped_controller_list_"));
        stopped_controller_list_->setGeometry(QRect(440, 20, 330, 487));
        stopped_controller_list_->setFont(font);
        stopped_controller_label = new QLabel(frame);
        stopped_controller_label->setObjectName(QString::fromUtf8("stopped_controller_label"));
        stopped_controller_label->setGeometry(QRect(442, 4, 330, 16));
        stopped_controller_label->setFont(font1);
        start_button_ = new QPushButton(frame);
        start_button_->setObjectName(QString::fromUtf8("start_button_"));
        start_button_->setEnabled(false);
        start_button_->setGeometry(QRect(344, 52, 90, 27));
        start_button_->setFont(font);
        stop_button_ = new QPushButton(frame);
        stop_button_->setObjectName(QString::fromUtf8("stop_button_"));
        stop_button_->setEnabled(false);
        stop_button_->setGeometry(QRect(344, 84, 90, 27));
        stop_button_->setFont(font);
        reload_button_ = new QPushButton(frame);
        reload_button_->setObjectName(QString::fromUtf8("reload_button_"));
        reload_button_->setEnabled(true);
        reload_button_->setGeometry(QRect(344, 184, 90, 27));
        reload_button_->setFont(font);

        retranslateUi(pr2_dynamic_movement_primitive_gui);

        QMetaObject::connectSlotsByName(pr2_dynamic_movement_primitive_gui);
    } // setupUi

    void retranslateUi(QWidget *pr2_dynamic_movement_primitive_gui)
    {
        pr2_dynamic_movement_primitive_gui->setWindowTitle(QApplication::translate("pr2_dynamic_movement_primitive_gui", "PR2 Dynamic Movement Primitive GUI", 0, QApplication::UnicodeUTF8));
        running_controller_label->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "Running controllers", 0, QApplication::UnicodeUTF8));
        execute_button_->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "exe&cute", 0, QApplication::UnicodeUTF8));
        update_button_->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "up&date", 0, QApplication::UnicodeUTF8));
        stopped_controller_label->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "Stopped controllers", 0, QApplication::UnicodeUTF8));
        start_button_->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "<-- st&art", 0, QApplication::UnicodeUTF8));
        stop_button_->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "st&op -->", 0, QApplication::UnicodeUTF8));
        reload_button_->setText(QApplication::translate("pr2_dynamic_movement_primitive_gui", "re&load", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class pr2_dynamic_movement_primitive_gui: public Ui_pr2_dynamic_movement_primitive_gui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // PR2_DYNAMIC_MOVEMENT_PRIMITIVE_GUI_MAIN_WINDOW_H
