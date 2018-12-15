#include "shortcutswindow.h"
#include "ui_shortcutswindow.h"

ShortcutsWindow::ShortcutsWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ShortcutsWindow)
{
    ui->setupUi(this);
}

ShortcutsWindow::~ShortcutsWindow()
{
    delete ui;
}

void ShortcutsWindow::on_pushButton_clicked()
{
    this->close();
}
