#include "widget.h"
#include <QApplication>

#include <QVTKOpenGLStereoWidget.h>
#include <qsurfaceformat.h>

int main(int argc, char* argv[]) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLStereoWidget::defaultFormat());

    QApplication a(argc, argv);
    Widget w;
    w.show();
    return a.exec();
}
