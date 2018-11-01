#include "mainwindow.h"
#include "ui_mainwindowform.h"
#include "osgwidget.h"

#include <QDockWidget>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow{parent},
  mMainWindowUI{ new Ui::MainWindowForm },
  mOSGWidget{ new OSGWidget{ this } }
{
  mMainWindowUI->setupUi(this);
  this->setCentralWidget(mOSGWidget);
}

MainWindow::~MainWindow()
{
  delete mMainWindowUI;
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::quit();
}

