#ifndef PODOGUI_BASE_H
#define PODOGUI_BASE_H

#include <QWidget>

namespace Ui {
class PODOGUI_Base;
}

class PODOGUI_Base : public QWidget
{
    Q_OBJECT

public:
    explicit PODOGUI_Base(QWidget *parent = 0);
    ~PODOGUI_Base();

private:
    Ui::PODOGUI_Base *ui;

    int ALNum;
};

#endif // PODOGUI_BASE_H
