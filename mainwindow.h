#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QtCore>

class OSGWidget;

namespace Ui
{
class MainWindowForm;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

public slots:
  void on_actionExit_triggered();

  void keyPressEvent(QKeyEvent *event);

protected:
  Ui::MainWindowForm *main_window_ui_;
  OSGWidget *osg_widget_{ nullptr };
};

#endif  // MAINWINDOW_H
