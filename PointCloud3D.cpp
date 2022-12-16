#include "PointCloud3D.hpp"
#include "csv2/reader.hpp"
#include "mesh.hpp"
#include "timer.hpp"
#include "ui_PointCloud3D.h"

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

#include <QFile>
#include <QFileDialog>
#include <QString>
#include <format>

namespace {
double interpolate(double value, double y0, double x0, double y1, double x1) {
    if (value < x0)
        return y0;
    if (value > x1)
        return y1;
    return (value - x0) * (y1 - y0) / (x1 - x0) + y0;
}

double jet_base(double value) {
    if (value <= -0.75)
        return 0.0;
    else if (value <= -0.25)
        return interpolate(value, 0.0, -0.75, 1.0, -0.25);
    else if (value <= 0.25)
        return 1.0;
    else if (value <= 0.75)
        return interpolate(value, 1.0, 0.25, 0.0, 0.75);
    else
        return 0.0;
}

std::array<double, 3> map_color(double value) {
    return std::array{
        jet_base(value * 2.0 - 1.5), // red
        jet_base(value * 2.0 - 1.0), // green
        jet_base(value * 2.0 - 0.5)  // blue
    };
}

bool equal(float v1, float v2) { return std::abs(v1 - v2) <= std::numeric_limits<float>::epsilon(); }

std::vector<std::array<float, 5>> read_data(const std::string& filename) {
    csv2::Reader<csv2::delimiter<','>, csv2::quote_character<'"'>, csv2::first_row_is_header<true>,
                 csv2::trim_policy::trim_whitespace>
        csv;

    if (csv.mmap(filename)) {
        const auto header = csv.header();
        if (csv.cols() != 5) {
            return {};
        }

        std::vector<std::array<float, 5>> result;
        result.reserve(csv.rows());
        for (const auto row : csv) {
            std::array<float, 5> row_val;
            std::size_t id{};
            for (const auto cell : row) {
                std::string value;
                cell.read_value(value);
                row_val[id] = std::stof(value);
                ++id;
            }
            result.push_back(row_val);
        }

        return result;
    }

    return {};
}
} // namespace

class PointCloud3D::PointCloud3DPrivate {
public:
    PointCloud3DPrivate()
        : points_{vtkSmartPointer<vtkPoints>::New()}
        , points_colors_{vtkSmartPointer<vtkUnsignedCharArray>::New()}
        , rail_vertices_{vtkSmartPointer<vtkCellArray>::New()}
        , defect_vertices_{vtkSmartPointer<vtkCellArray>::New()}
        , triangle_array_{vtkSmartPointer<vtkCellArray>::New()}
        , rail_poly_data_{vtkSmartPointer<vtkPolyData>::New()}
        , defect_poly_data_{vtkSmartPointer<vtkPolyData>::New()}
        , rail_norm_filter_{vtkSmartPointer<vtkPolyDataNormals>::New()}
        , rail_mapper_{vtkSmartPointer<vtkPolyDataMapper>::New()}
        , defect_mapper_{vtkSmartPointer<vtkPolyDataMapper>::New()}
        , rail_actor_{vtkSmartPointer<vtkActor>::New()}
        , defect_actor_{vtkSmartPointer<vtkActor>::New()}
        , axes_actor_{vtkSmartPointer<vtkCubeAxesActor>::New()}
        , renderer_{vtkSmartPointer<vtkRenderer>::New()}
        , render_window_{vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()} {}

private:
    vtkSmartPointer<vtkPoints> points_;                   // 所有顶点
    vtkSmartPointer<vtkUnsignedCharArray> points_colors_; // 所有顶点颜色

    vtkSmartPointer<vtkCellArray> rail_vertices_;          // 需要绘制的钢轨顶点索引
    vtkSmartPointer<vtkCellArray> defect_vertices_;        // 需要绘制的缺陷顶点索引
    vtkSmartPointer<vtkCellArray> triangle_array_;         // 绘制钢轨的三角形
    vtkSmartPointer<vtkPolyData> rail_poly_data_;          // 多边形数据集(钢轨)
    vtkSmartPointer<vtkPolyData> defect_poly_data_;        // 多边形数据集(缺陷)
    vtkSmartPointer<vtkPolyDataNormals> rail_norm_filter_; // 计算法向量
    vtkSmartPointer<vtkPolyDataMapper> rail_mapper_;       // 图形数据到渲染图元的转换
    vtkSmartPointer<vtkPolyDataMapper> defect_mapper_;     // 图形数据到渲染图元的转换

    vtkSmartPointer<vtkActor> rail_actor_;
    vtkSmartPointer<vtkActor> defect_actor_;
    vtkSmartPointer<vtkCubeAxesActor> axes_actor_;

    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> render_window_;

    friend class PointCloud3D;
};

PointCloud3D::PointCloud3D(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::PointCloud3D)
    , d{std::make_unique <PointCloud3DPrivate>()} {
    ui->setupUi(this);

    // 指定(颜色)数组中每个元组的大小
    d->points_colors_->SetNumberOfComponents(3);

    init_geometry_rail();
    init_geometry_defect();
    init_axis();

    // 设置背景色
    const vtkColor3d bgcolor{0.18, 0.22, 0.25};
    d->renderer_->SetBackground(bgcolor.GetData());
    d->renderer_->ResetCamera();
    d->renderer_->ResetCameraClippingRange();

    // 渲染
    d->render_window_->AddRenderer(d->renderer_);
    ui->qvtkWidget->setRenderWindow(d->render_window_);
    d->render_window_->Render();

    connect(ui->selectBtn, &QPushButton::clicked, this, [this]() {
        const QString filename = QFileDialog::getOpenFileName();
        if (filename.isEmpty())
            return;
        ui->fileLineEdit->setText(filename);
        const auto datas = read_data(filename.toStdString());
        update_data(datas);
    });
    connect(ui->axisCheckBox, &QCheckBox::stateChanged, this, [this]() {
        d->axes_actor_->SetVisibility(ui->axisCheckBox->isChecked());
        d->render_window_->Render();
    });
    connect(ui->modeComboBox, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (ui->modeComboBox->currentIndex() == 0) {
            d->rail_poly_data_->SetPolys(d->triangle_array_); // 渲染三角形
            d->rail_poly_data_->SetVerts({});                 // 设置渲染顶点
            d->rail_poly_data_->GetPointData()->SetScalars({}); //设置顶点颜色
        } else {
            d->rail_poly_data_->SetPolys({});
            d->rail_poly_data_->SetVerts(d->rail_vertices_);                // 设置渲染顶点
            d->rail_poly_data_->GetPointData()->SetScalars(d->points_colors_); //设置顶点颜色
        }

        d->rail_poly_data_->Modified();
        d->rail_norm_filter_->Update();
        d->render_window_->Render();
    });
    connect(ui->transparencySlider, &QSlider::valueChanged, this, [this](int val) {
        d->rail_actor_->GetProperty()->SetOpacity(1.0 - val / 100.0);
        d->render_window_->Render();
    });
    connect(ui->pointSizeSpinBox, &QSpinBox::valueChanged, this, [this](int val) {
        d->defect_actor_->GetProperty()->SetPointSize(val);
        d->render_window_->Render();
    });
    connect(ui->ambientSpinBox, &QDoubleSpinBox::valueChanged, this, [this](int val) {
        d->rail_actor_->GetProperty()->SetAmbient(val); // 环境光系数
        d->render_window_->Render();
    });
    connect(ui->diffuseSpinBox, &QDoubleSpinBox::valueChanged, this, [this](int val) {
        d->rail_actor_->GetProperty()->SetDiffuse(val); // 漫反射光系数
        d->render_window_->Render();
    });
    connect(ui->specularSpinBox, &QDoubleSpinBox::valueChanged, this, [this](int val) {
        d->rail_actor_->GetProperty()->SetSpecular(val); // 镜反射光系数
        d->render_window_->Render();
    });
    connect(ui->specularPowerSpinBox, &QSpinBox::valueChanged, this, [this](int val) {
        d->rail_actor_->GetProperty()->SetSpecularPower(val); // 高光指数
        d->render_window_->Render();
    });
    connect(ui->setPositionBtn, &QPushButton::clicked, this, [this](int val) {
        auto camera = d->renderer_->GetActiveCamera();
        camera->SetPosition(ui->positionXSpinBox->value(), ui->positionYSpinBox->value(),
                            ui->positionZSpinBox->value());
        d->render_window_->Render();
    });
    connect(ui->getPositionBtn, &QPushButton::clicked, this, [this](int val) {
        const auto camera = d->renderer_->GetActiveCamera();
        const double* position = camera->GetPosition();
        ui->positionXSpinBox->setValue(position[0]);
        ui->positionYSpinBox->setValue(position[1]);
        ui->positionZSpinBox->setValue(position[2]);
    });
}

PointCloud3D::~PointCloud3D() { delete ui; }

void PointCloud3D::update_data(const std::vector<std::array<float, 5>>& cloud) {
    printf("Start update data\n");
    timer t;
    d->points_->Resize(0);
    d->points_colors_->Resize(0);
    d->rail_vertices_->Reset();
    d->defect_vertices_->Reset();

    // 物体
    const int max_point_count = cloud.size();

    Mesh mesh(10);
    mesh.reserve(max_point_count);

    // 读点云数据信息
    float prev_z = std::numeric_limits<float>::lowest();
    std::vector<Point3D> section;
    section.reserve(4096);
    std::size_t point_count{};
    for (int n = 0; n < max_point_count; ++n) {
        const auto& value = cloud[n];
        ++point_count;

        d->points_->InsertNextPoint(value[0], value[1], value[2]); // 加入点信息

        const auto colors = map_color(value[3]);
        unsigned char color[] = {colors[0] * 255, colors[1] * 255, colors[2] * 255};
        d->points_colors_->InsertNextTypedTuple(color);

        if (value[4] < 1) { // 钢轨
            if (!equal(value[2], prev_z)) {
                mesh.add_section(section);
                section.clear();
                prev_z = value[2];
            }
            section.push_back({value[0], value[1], value[2]});

            d->rail_vertices_->InsertNextCell(1); // 加入顶点信息----用于渲染点集
            d->rail_vertices_->InsertCellPoint(n);

        } else {                                 // 缺陷
            d->defect_vertices_->InsertNextCell(1); // 加入顶点信息----用于渲染点集
            d->defect_vertices_->InsertCellPoint(n);
        }
    }

    // 三角形
    const auto t1 = t.restart();
    d->triangle_array_->Reset();
    d->triangle_array_->AllocateEstimate(mesh.triangles().size(), sizeof(TrianglePointId));
    for (auto& triangle : mesh.triangles()) {
        d->triangle_array_->InsertNextCell({triangle[0], triangle[1], triangle[2]});
    }
    printf("Time: %lf, %lf\n", t1, t.elapsed());

    d->rail_poly_data_->SetPoints(d->points_); // 设置点集
    if (ui->modeComboBox->currentIndex() == 0) {
        d->rail_poly_data_->SetPolys(d->triangle_array_); // 渲染三角形
    } else {
        d->rail_poly_data_->SetVerts(d->rail_vertices_);                // 设置渲染顶点
        d->rail_poly_data_->GetPointData()->SetScalars(d->points_colors_); //设置顶点颜色
    }

    d->defect_poly_data_->SetPoints(d->points_);                   // 设置点集
    d->defect_poly_data_->SetVerts(d->defect_vertices_);           // 设置渲染顶点
    d->defect_poly_data_->GetPointData()->SetScalars(d->points_colors_); //设置顶点颜色

    d->axes_actor_->SetBounds(d->points_->GetBounds());

    d->rail_poly_data_->Modified();
    d->defect_poly_data_->Modified();
    d->rail_norm_filter_->Update();
    d->renderer_->ResetCamera();
    d->render_window_->Render();
}

void PointCloud3D::init_geometry_rail() {
    // 计算法几何体向量
    d->rail_norm_filter_->SetInputData(d->rail_poly_data_);
    d->rail_norm_filter_->SetComputePointNormals(1);
    d->rail_norm_filter_->SetComputeCellNormals(1);

    // 映射器负责将几何体推入图形库。如果定义了标量或其他属性，它还可以进行颜色映射。
    d->rail_mapper_->SetInputData(d->rail_norm_filter_->GetOutput());

    // actor是一种分组机制：除了几何体（mapper），它还具有属性、变换矩阵和/或纹理贴图
    d->rail_actor_->SetMapper(d->rail_mapper_);
    d->rail_actor_->GetProperty()->SetColor(0.87, 0.87, 0.87);
    d->rail_actor_->GetProperty()->SetPointSize(1);
    d->rail_actor_->GetProperty()->SetAmbient(0.4);     // 环境光系数
    d->rail_actor_->GetProperty()->SetDiffuse(0.4);     // 漫反射光系数
    d->rail_actor_->GetProperty()->SetSpecular(0.2);    // 镜反射光系数
    d->rail_actor_->GetProperty()->SetSpecularPower(20.0); // 镜面高光指数

    d->rail_actor_->GetProperty()->SetAmbientColor(74.0 / 255.0, 171.0 / 255.0, 255.0 / 225.0);
    d->rail_actor_->GetProperty()->SetDiffuseColor(74.0 / 255.0, 171.0 / 255.0, 255.0 / 225.0);
    d->rail_actor_->GetProperty()->SetSpecularColor(74.0 / 255.0, 171.0 / 255.0, 255.0 / 225.0);

    d->renderer_->AddActor(d->rail_actor_);
}

void PointCloud3D::init_geometry_defect() {
    d->defect_mapper_->SetInputData(d->defect_poly_data_);

    d->defect_actor_->SetMapper(d->defect_mapper_);
    d->defect_actor_->GetProperty()->SetPointSize(1);

    d->renderer_->AddActor(d->defect_actor_);
}

void PointCloud3D::init_axis() {
    d->axes_actor_->SetCamera(d->renderer_->GetActiveCamera());
    // 设置x、y、z轴的起始和终止值
    d->axes_actor_->SetBounds(d->points_->GetBounds());
    // 设置坐标轴线的宽度
    d->axes_actor_->GetXAxesLinesProperty()->SetLineWidth(0.5);
    d->axes_actor_->GetYAxesLinesProperty()->SetLineWidth(0.5);
    d->axes_actor_->GetZAxesLinesProperty()->SetLineWidth(0.5);
    // 设置标题和标签文本的屏幕大小。默认值为10.0。
    d->axes_actor_->SetScreenSize(6);
    // 指定标签与轴之间的距离。默认值为20.0。
    d->axes_actor_->SetLabelOffset(5);
    //显示坐标轴
    d->axes_actor_->SetVisibility(true);
    //指定一种模式来控制轴的绘制方式
    d->axes_actor_->SetFlyMode(0);
    //设置惯性因子，该惯性因子控制轴切换位置的频率（从一个轴跳到另一个轴）
    //d->axes_actor_->SetInertia(1);

    //网格设置
    //开启x、y、z轴的网格线绘制
    d->axes_actor_->DrawXGridlinesOn();
    d->axes_actor_->DrawYGridlinesOn();
    d->axes_actor_->DrawZGridlinesOn();
    //设置x、y、z轴的内部网格线不绘制
    d->axes_actor_->SetDrawXInnerGridlines(false);
    d->axes_actor_->SetDrawYInnerGridlines(false);
    d->axes_actor_->SetDrawZInnerGridlines(false);
    //设置x、y、z轴网格线的颜色
    d->axes_actor_->GetXAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    d->axes_actor_->GetYAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    d->axes_actor_->GetZAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    //指定网格线呈现的样式
    d->axes_actor_->SetGridLineLocation(2);

    //刻度的设置
    //不显示x、y、z轴的次刻度
    d->axes_actor_->XAxisMinorTickVisibilityOff();
    d->axes_actor_->YAxisMinorTickVisibilityOff();
    d->axes_actor_->ZAxisMinorTickVisibilityOff();
    //设置刻度标签的显示方式(参数1为false，刻度标签按0-200000显示；为true时，按0-200显示)
    d->axes_actor_->SetLabelScaling(false, 0, 0, 0);
    //设置刻度线显示的位置(内部、外部、两侧)
    d->axes_actor_->SetTickLocation(1);

    d->renderer_->AddActor(d->axes_actor_);
}
