#pragma once

#include <memory>
#include <QWidget>

namespace Ui {
class PointCloud3D;
}

class PointCloud3D : public QWidget {
    Q_OBJECT

public:
    explicit PointCloud3D(QWidget* parent = nullptr);
    ~PointCloud3D();
    void update_data(const std::vector<std::array<float, 5>>& cloud);

private:
    // 几何体
    void init_geometry_rail();
    void init_geometry_defect();
    // 坐标轴
    void init_axis();

private:
    Ui::PointCloud3D* ui;
    class PointCloud3DPrivate;
    std::unique_ptr<PointCloud3DPrivate> d;
};
