#include "PointCloud3D.hpp"
#include <QApplication>

#include <QVTKOpenGLStereoWidget.h>
#include <qsurfaceformat.h>

int main(int argc, char* argv[]) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLStereoWidget::defaultFormat());

    QApplication a(argc, argv);
    PointCloud3D w;
    w.show();
    return a.exec();
}
