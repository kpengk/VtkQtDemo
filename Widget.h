#pragma once

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCellData.h>
#include <vtkCubeAxesActor.h>
#include <vtkCubeSource.h>
#include <vtkFloatArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkLight.h>
#include <vtkLookupTable.h>
#include <vtkNamedColors.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTriangle.h>
#include <QWidget>


namespace Ui {
class Widget;
}

class Widget : public QWidget {
    Q_OBJECT

public:
    explicit Widget(QWidget* parent = 0);
    ~Widget();
    void update_data(const std::vector<std::array<float, 5>>& cloud);

private:
    // 几何体
    void init_geometry_rail();
    void init_geometry_defect();
    // 坐标轴
    void init_axis();

private:
    Ui::Widget* ui;

    vtkSmartPointer<vtkPoints> points_;// 所有顶点
    vtkSmartPointer<vtkUnsignedCharArray> points_colors_; // 所有顶点颜色

    vtkSmartPointer<vtkCellArray> rail_vertices_;// 需要绘制的钢轨顶点索引
    vtkSmartPointer<vtkCellArray> defect_vertices_;// 需要绘制的缺陷顶点索引
    vtkSmartPointer<vtkCellArray> triangle_array_;// 绘制钢轨的三角形
    vtkSmartPointer<vtkPolyData> rail_poly_data_;// 多边形数据集(钢轨)
    vtkSmartPointer<vtkPolyData> defect_poly_data_;// 多边形数据集(缺陷)
    vtkSmartPointer<vtkPolyDataNormals> rail_norm_filter_;// 计算法向量
    vtkSmartPointer<vtkPolyDataMapper> rail_mapper_;// 图形数据到渲染图元的转换
    vtkSmartPointer<vtkPolyDataMapper> defect_mapper_;// 图形数据到渲染图元的转换

    vtkSmartPointer<vtkActor> rail_actor_;
    vtkSmartPointer<vtkActor> defect_actor_;
    vtkSmartPointer<vtkCubeAxesActor> axes_actor_;

    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> render_window_;

    vtkColor4d default_color_;
};
