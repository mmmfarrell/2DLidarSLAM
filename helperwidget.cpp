#include "helperwidget.h"
#include "ui_helperwidget.h"

HelperWidget::HelperWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HelperWidget)
{
    ui->setupUi(this);
}

HelperWidget::~HelperWidget()
{
    delete ui;
}

void HelperWidget::updateRobotVelocityFactor(int new_robot_speed)
{
  ui->spinBox->setValue(new_robot_speed);
}

void HelperWidget::on_spinBox_valueChanged(int new_robot_speed)
{
  emit scaleFactorChanged(new_robot_speed);
}

void HelperWidget::on_resetMapButton_clicked()
{
  emit resetMap();
}

void HelperWidget::on_resetSlamButton_clicked()
{
  emit resetSlamMap();
}

void HelperWidget::on_resetRobotButton_clicked()
{
  emit resetRobotPose();
}
