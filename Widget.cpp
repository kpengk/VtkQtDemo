#include "widget.h"
#include "mesh.hpp"
#include "ui_widget.h"
#include "timer.hpp"
#include "csv2/reader.hpp"

#include <format>
#include <QFileDialog>
#include <QFile>
#include <QString>

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
    csv2::Reader<csv2::delimiter<','>, csv2::quote_character<'"'>,
        csv2::first_row_is_header<true>,
        csv2::trim_policy::trim_whitespace> csv;

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

// https://blog.csdn.net/qq_35769071/article/details/122671756

Widget::Widget(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , points_{vtkSmartPointer<vtkPoints>::New()}
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
    , render_window_{vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()} {
    ui->setupUi(this);

    // 指定(颜色)数组中每个元组的大小
    points_colors_->SetNumberOfComponents(3);

    init_geometry_rail();
    init_geometry_defect();
    init_axis();
    

    // 设置背景色
    const vtkColor3d bgcolor{0.18, 0.22, 0.25};
    renderer_->SetBackground(bgcolor.GetData());
    renderer_->ResetCamera();
    renderer_->ResetCameraClippingRange();

    // 渲染
    render_window_->AddRenderer(renderer_);
    ui->qvtkWidget->setRenderWindow(render_window_);
    render_window_->Render();


    connect(ui->selectBtn, &QPushButton::clicked, this, [this]() {
        const QString filename = QFileDialog::getOpenFileName();
        if (filename.isEmpty())
            return;
        ui->fileLineEdit->setText(filename);
        const auto datas = read_data(filename.toStdString());
        update_data(datas);
    });
    connect(ui->axisCheckBox, &QCheckBox::stateChanged, this, [this]() {
        axes_actor_->SetVisibility(ui->axisCheckBox->isChecked());
        render_window_->Render();
    });
    connect(ui->modeComboBox, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (ui->modeComboBox->currentIndex() == 0) {
            rail_poly_data_->SetPolys(triangle_array_); // 渲染三角形
            rail_poly_data_->SetVerts({});                   // 设置渲染顶点
            rail_poly_data_->GetPointData()->SetScalars({}); //设置顶点颜色
        } else {
            rail_poly_data_->SetPolys({});
            rail_poly_data_->SetVerts(rail_vertices_);                   // 设置渲染顶点
            rail_poly_data_->GetPointData()->SetScalars(points_colors_); //设置顶点颜色
        }

        rail_poly_data_->Modified();
        rail_norm_filter_->Update();
        render_window_->Render();
    });
    connect(ui->transparencySlider, &QSlider::valueChanged, this, [this](int val) {
        rail_actor_->GetProperty()->SetOpacity(1.0 - val / 100.0);
        render_window_->Render();
    });
    connect(ui->pointSizeSpinBox, &QSpinBox::valueChanged, this, [this](int val){
        defect_actor_->GetProperty()->SetPointSize(val);
        render_window_->Render();
    });
    connect(ui->specularPowerSpinBox, &QSpinBox::valueChanged, this, [this](int val) {
        rail_actor_->GetProperty()->SetSpecularPower(val); // 镜面指数
        render_window_->Render();
    });
    connect(ui->setPositionBtn, &QPushButton::clicked, this, [this](int val) {
        auto camera = renderer_->GetActiveCamera();
        camera->SetPosition(ui->positionXSpinBox->value(), ui->positionYSpinBox->value(),
                            ui->positionZSpinBox->value());
        render_window_->Render();
    });
    connect(ui->getPositionBtn, &QPushButton::clicked, this, [this](int val) {
        const auto camera = renderer_->GetActiveCamera();
        const double* position = camera->GetPosition();
        ui->positionXSpinBox->setValue(position[0]);
        ui->positionYSpinBox->setValue(position[1]);
        ui->positionZSpinBox->setValue(position[2]);
    });
}

Widget::~Widget() { delete ui; }


void Widget::update_data(const std::vector<std::array<float, 5>>& cloud) {
    printf("Start update data\n");
    timer t;
    points_->Resize(0);
    points_colors_->Resize(0);
    rail_vertices_->Reset();
    defect_vertices_->Reset();

    // 物体
    const int max_point_count = cloud.size();

    Mesh mesh;
    mesh.reserve(max_point_count);

    // 读点云数据信息
    float prev_z = std::numeric_limits<float>::lowest();
    std::vector<Point3D> section;
    section.reserve(4096);
    std::size_t point_count{};
    for (int n = 0; n < max_point_count; ++n) {
        const auto& value = cloud[n];
        ++point_count;

        points_->InsertNextPoint(value[0], value[1], value[2]); // 加入点信息

        const auto colors = map_color(value[3]);
        unsigned char color[] = {colors[0] * 255, colors[1] * 255, colors[2] * 255};
        points_colors_->InsertNextTypedTuple(color);

        if (value[4] < 1) { // 钢轨
            if (!equal(value[2], prev_z)) {
                mesh.add_section(section);
                section.clear();
                prev_z = value[2];
            }
            section.push_back({value[0], value[1], value[2]});

            rail_vertices_->InsertNextCell(1); // 加入顶点信息----用于渲染点集
            rail_vertices_->InsertCellPoint(n);

        } else { // 缺陷
            defect_vertices_->InsertNextCell(1);// 加入顶点信息----用于渲染点集
            defect_vertices_->InsertCellPoint(n);
        }
    }

    // 三角形
    const auto t1 = t.restart();
    triangle_array_->Reset();
    triangle_array_->AllocateEstimate(mesh.triangles().size(), sizeof(TrianglePointId));
    for (auto& triangle : mesh.triangles()) {
        triangle_array_->InsertNextCell({triangle[0], triangle[1], triangle[2]});
    }
    printf("Time: %lf, %lf\n", t1, t.elapsed());

    rail_poly_data_->SetPoints(points_); // 设置点集
    if (ui->modeComboBox->currentIndex() == 0) {
        rail_poly_data_->SetPolys(triangle_array_);                            // 渲染三角形
    } else {
        rail_poly_data_->SetVerts(rail_vertices_);                   // 设置渲染顶点
        rail_poly_data_->GetPointData()->SetScalars(points_colors_); //设置顶点颜色
    }

    defect_poly_data_->SetPoints(points_);                         // 设置点集
    defect_poly_data_->SetVerts(defect_vertices_);                   // 设置渲染顶点
    defect_poly_data_->GetPointData()->SetScalars(points_colors_); //设置顶点颜色
    
    axes_actor_->SetBounds(points_->GetBounds());

    rail_poly_data_->Modified();
    defect_poly_data_->Modified();
    rail_norm_filter_->Update();
    renderer_->ResetCamera();
    render_window_->Render();
}

void Widget::init_geometry_rail() {
    // 计算法几何体向量
    rail_norm_filter_->SetInputData(rail_poly_data_);
    rail_norm_filter_->SetComputePointNormals(1);
    rail_norm_filter_->SetComputeCellNormals(1);

    // 映射器负责将几何体推入图形库。如果定义了标量或其他属性，它还可以进行颜色映射。
    rail_mapper_->SetInputData(rail_norm_filter_->GetOutput());

    // actor是一种分组机制：除了几何体（mapper），它还具有属性、变换矩阵和/或纹理贴图
    rail_actor_->SetMapper(rail_mapper_);
    rail_actor_->GetProperty()->SetColor(0.87, 0.87, 0.87);
    rail_actor_->GetProperty()->SetPointSize(1);
    rail_actor_->GetProperty()->SetAmbient(0.4);        // 环境光系数
    rail_actor_->GetProperty()->SetDiffuse(0.4);        // 漫反射光系数
    rail_actor_->GetProperty()->SetSpecular(0.2);       // 镜反射光系数
    rail_actor_->GetProperty()->SetSpecularPower(20.0);  // 镜面指数

    renderer_->AddActor(rail_actor_);
}

void Widget::init_geometry_defect() {
    // 映射器负责将几何体推入图形库。如果定义了标量或其他属性，它还可以进行颜色映射。
    defect_mapper_->SetInputData(defect_poly_data_);

    // actor是一种分组机制：除了几何体（mapper），它还具有属性、变换矩阵和/或纹理贴图
    defect_actor_->SetMapper(defect_mapper_);
    defect_actor_->GetProperty()->SetPointSize(1);
    defect_actor_->GetProperty()->SetAmbient(0.4);     // 环境光系数
    defect_actor_->GetProperty()->SetDiffuse(0.4);     // 漫反射光系数
    defect_actor_->GetProperty()->SetSpecular(0.2);    // 镜反射光系数
    defect_actor_->GetProperty()->SetSpecularPower(20.0); // 镜面指数

    renderer_->AddActor(defect_actor_);
}

void Widget::init_axis() {
    axes_actor_->SetCamera(renderer_->GetActiveCamera());
    // 设置x、y、z轴的起始和终止值
    axes_actor_->SetBounds(points_->GetBounds());
    // 设置坐标轴线的宽度
    axes_actor_->GetXAxesLinesProperty()->SetLineWidth(0.5);
    axes_actor_->GetYAxesLinesProperty()->SetLineWidth(0.5);
    axes_actor_->GetZAxesLinesProperty()->SetLineWidth(0.5);
    // 设置标题和标签文本的屏幕大小。默认值为10.0。
    axes_actor_->SetScreenSize(6);
    // 指定标签与轴之间的距离。默认值为20.0。
    axes_actor_->SetLabelOffset(5);
    //显示坐标轴
    axes_actor_->SetVisibility(true);
    //指定一种模式来控制轴的绘制方式
    axes_actor_->SetFlyMode(0);
    //设置惯性因子，该惯性因子控制轴切换位置的频率（从一个轴跳到另一个轴）
    // m_cubeAxesActor->SetInertia(1);

    //网格设置
    //开启x、y、z轴的网格线绘制
    axes_actor_->DrawXGridlinesOn();
    axes_actor_->DrawYGridlinesOn();
    axes_actor_->DrawZGridlinesOn();
    //设置x、y、z轴的内部网格线不绘制
    axes_actor_->SetDrawXInnerGridlines(false);
    axes_actor_->SetDrawYInnerGridlines(false);
    axes_actor_->SetDrawZInnerGridlines(false);
    //设置x、y、z轴网格线的颜色
    axes_actor_->GetXAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    axes_actor_->GetYAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    axes_actor_->GetZAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    //指定网格线呈现的样式
    axes_actor_->SetGridLineLocation(2);

    //刻度的设置
    //不显示x、y、z轴的次刻度
    axes_actor_->XAxisMinorTickVisibilityOff();
    axes_actor_->YAxisMinorTickVisibilityOff();
    axes_actor_->ZAxisMinorTickVisibilityOff();
    //设置刻度标签的显示方式(参数1为false，刻度标签按0-200000显示；为true时，按0-200显示)
    axes_actor_->SetLabelScaling(false, 0, 0, 0);
    //设置刻度线显示的位置(内部、外部、两侧)
    axes_actor_->SetTickLocation(1);

    renderer_->AddActor(axes_actor_);
}
