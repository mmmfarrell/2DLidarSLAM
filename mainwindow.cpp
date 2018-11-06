#include "mainwindow.h"
#include "ui_mainwindowform.h"
#include "osgwidget.h"

#include <iostream> // TODO remove
#include <QDockWidget>
#include <QKeyEvent>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow{parent},
  main_window_ui_{ new Ui::MainWindowForm },
  osg_widget_{ new OSGWidget{ this } }
{
  main_window_ui_->setupUi(this);
  this->setCentralWidget(osg_widget_);
}

MainWindow::~MainWindow()
{
  delete main_window_ui_;
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::quit();
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
  switch (event->key())
  {
    case Qt::Key_Up:
      std::cout << "Up" << std::endl;
      break;
    case Qt::Key_Escape:
    case Qt::Key_Q:
      QApplication::quit();
      break;
  }

  QMainWindow::keyPressEvent(event);
}
