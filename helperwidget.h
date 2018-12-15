#ifndef HELPERWIDGET_H
#define HELPERWIDGET_H

#include <QWidget>

namespace Ui {
class HelperWidget;
}

class HelperWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HelperWidget(QWidget *parent = 0);
    ~HelperWidget();
    void updateRobotVelocityFactor(int new_robot_speed);

private slots:
    void on_spinBox_valueChanged(int new_robot_speed);
    void on_resetMapButton_clicked();
    void on_resetSlamButton_clicked();
    void on_resetRobotButton_clicked();

signals:
    void scaleFactorChanged(int new_scale_factor);
    void resetMap();
    void resetSlamMap();
    void resetRobotPose();

private:
    Ui::HelperWidget *ui;
};

#endif // HELPERWIDGET_H
