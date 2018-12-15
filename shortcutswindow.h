#ifndef SHORTCUTSWINDOW_H
#define SHORTCUTSWINDOW_H

#include <QDialog>

namespace Ui {
class ShortcutsWindow;
}

class ShortcutsWindow : public QDialog
{
    Q_OBJECT

public:
    explicit ShortcutsWindow(QWidget *parent = nullptr);
    ~ShortcutsWindow();

private slots:
    void on_pushButton_clicked();

private:
    Ui::ShortcutsWindow *ui;
};

#endif // SHORTCUTSWINDOW_H
