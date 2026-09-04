#include "omx_box_control/omx_target_panel.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <sstream>
#include <iomanip>
#include <set>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace omx_box_control
{

OmxTargetPanel::OmxTargetPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * form = new QFormLayout();
  x_input_ = makePositionInput(0.20);
  y_input_ = makePositionInput(0.00);
  z_input_ = makePositionInput(0.15);
  form->addRow("X (m)", x_input_);
  form->addRow("Y (m)", y_input_);
  form->addRow("Z (m)", z_input_);

  move_button_ = new QPushButton("Move");
  move_button_->setEnabled(false);
  workspace_button_ = new QPushButton("Show Reachable Grid");
  workspace_button_->setCheckable(true);
  workspace_button_->setChecked(true);
  workspace_button_->setEnabled(false);
  status_label_ = new QLabel("Waiting for RViz initialization");
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout();
  layout->addLayout(form);
  layout->addWidget(move_button_);
  layout->addWidget(workspace_button_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(move_button_, &QPushButton::clicked, this, &OmxTargetPanel::publishTarget);
  connect(workspace_button_, &QPushButton::toggled, this, &OmxTargetPanel::toggleWorkspace);
  connect(x_input_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &OmxTargetPanel::configChanged);
  connect(y_input_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &OmxTargetPanel::configChanged);
  connect(z_input_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &OmxTargetPanel::configChanged);
}

QDoubleSpinBox * OmxTargetPanel::makePositionInput(double value)
{
  auto * input = new QDoubleSpinBox();
  input->setDecimals(3);
  input->setSingleStep(0.01);
  input->setRange(-0.50, 0.50);
  input->setValue(value);
  return input;
}

void OmxTargetPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("Failed to access the RViz ROS node");
    return;
  }
  node_ = abstraction->get_raw_node();
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/box_target_pose", 10);
  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/box_target_marker", rclcpp::QoS(1).transient_local().reliable());
  move_button_->setEnabled(true);
  workspace_button_->setEnabled(true);
  toggleWorkspace(true);
  status_label_->setText("Ready: /box_target_pose (frame: link0)");
}

void OmxTargetPanel::publishTarget()
{
  if (!publisher_) {
    status_label_->setText("Publisher is not initialized");
    return;
  }

  geometry_msgs::msg::PoseStamped target;
  target.header.stamp = node_->now();
  target.header.frame_id = "link0";
  target.pose.position.x = x_input_->value();
  target.pose.position.y = y_input_->value();
  target.pose.position.z = z_input_->value();
  target.pose.orientation.w = 1.0;
  publisher_->publish(target);
  publishTargetMarkers(target);

  status_label_->setText(
    QString("Sent: x=%1, y=%2, z=%3")
    .arg(target.pose.position.x, 0, 'f', 3)
    .arg(target.pose.position.y, 0, 'f', 3)
    .arg(target.pose.position.z, 0, 'f', 3));
}


void OmxTargetPanel::toggleWorkspace(bool visible)
{
  if (!marker_publisher_) {return;}

  visualization_msgs::msg::MarkerArray array;
  if (!visible) {
    for (int id : {10, 11}) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "link0";
      marker.header.stamp = node_->now();
      marker.ns = "omx_reachable_workspace";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(marker);
    }
    marker_publisher_->publish(array);
    workspace_button_->setText("Show Reachable Grid");
    return;
  }

  visualization_msgs::msg::Marker grid;
  grid.header.frame_id = "link0";
  grid.header.stamp = node_->now();
  grid.ns = "omx_reachable_workspace";
  grid.id = 10;
  grid.type = visualization_msgs::msg::Marker::POINTS;
  grid.action = visualization_msgs::msg::Marker::ADD;
  grid.pose.orientation.w = 1.0;
  grid.scale.x = 0.009;
  grid.scale.y = 0.009;
  grid.color.a = 1.0;

  constexpr double q2_min = -2.0944;
  constexpr double q2_max = 1.5708;
  constexpr double q3_min = -2.0944;
  constexpr double q3_max = 1.5708;
  constexpr double q4_limit = 1.74533;
  constexpr double pi = 3.14159265358979323846;
  constexpr double voxel = 0.012;
  std::set<std::string> occupied;

  for (int yaw_i = 0; yaw_i <= 48; ++yaw_i) {
    const double q1 = -pi + 2.0 * pi * yaw_i / 48.0;
    const double c1 = std::cos(q1);
    const double s1 = std::sin(q1);
    for (int q2_i = 0; q2_i <= 40; ++q2_i) {
      const double q2 = q2_min + (q2_max - q2_min) * q2_i / 40.0;
      for (int q3_i = 0; q3_i <= 40; ++q3_i) {
        const double q3 = q3_min + (q3_max - q3_min) * q3_i / 40.0;
        const double q4 = -(q2 + q3);
        if (q4 < -q4_limit || q4 > q4_limit) {continue;}

        // OMX-F forward kinematics with horizontal tool pitch (q2+q3+q4=0).
        const double radial =
          0.0415 * std::cos(q2) + 0.11315 * std::sin(q2) +
          0.162 * std::cos(q2 + q3) + 0.0287 + 0.09193;
        const double z =
          0.0975 - 0.0415 * std::sin(q2) + 0.11315 * std::cos(q2) -
          0.162 * std::sin(q2 + q3);
        const double x = -0.01125 + radial * c1 + 0.0016 * s1;
        const double y = radial * s1 - 0.0016 * c1;
        if (x < 0.08 || x > 0.32 || std::abs(y) > 0.25 || z < 0.01 || z > 0.32) {
          continue;
        }

        const int vx = static_cast<int>(std::lround(x / voxel));
        const int vy = static_cast<int>(std::lround(y / voxel));
        const int vz = static_cast<int>(std::lround(z / voxel));
        const std::string key = std::to_string(vx) + ":" + std::to_string(vy) + ":" +
          std::to_string(vz);
        if (!occupied.insert(key).second) {continue;}

        geometry_msgs::msg::Point point;
        point.x = vx * voxel;
        point.y = vy * voxel;
        point.z = vz * voxel;
        grid.points.push_back(point);

        const double margin2 = std::min(q2 - q2_min, q2_max - q2);
        const double margin3 = std::min(q3 - q3_min, q3_max - q3);
        const double margin4 = q4_limit - std::abs(q4);
        std_msgs::msg::ColorRGBA color;
        if (std::min({margin2, margin3, margin4}) < 0.20) {
          color.r = 1.0F; color.g = 0.75F; color.b = 0.0F; color.a = 0.30F;
        } else {
          color.r = 0.1F; color.g = 1.0F; color.b = 0.25F; color.a = 0.22F;
        }
        grid.colors.push_back(color);
      }
    }
  }
  array.markers.push_back(grid);

  visualization_msgs::msg::Marker label;
  label.header = grid.header;
  label.ns = grid.ns;
  label.id = 11;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.position.x = 0.10;
  label.pose.position.y = 0.0;
  label.pose.position.z = 0.34;
  label.pose.orientation.w = 1.0;
  label.scale.z = 0.022;
  label.color.r = 0.7F; label.color.g = 1.0F; label.color.b = 0.7F; label.color.a = 0.9F;
  label.text = "Horizontal reachable workspace (approx.)";
  array.markers.push_back(label);
  marker_publisher_->publish(array);
  workspace_button_->setText("Hide Reachable Grid");
}

void OmxTargetPanel::publishTargetMarkers(const geometry_msgs::msg::PoseStamped & target)
{
  visualization_msgs::msg::MarkerArray array;
  const std::array<std::array<float, 3>, 3> colors = {{{1.0F, 0.0F, 0.0F},
    {0.0F, 1.0F, 0.0F}, {0.0F, 0.4F, 1.0F}}};

  for (int axis = 0; axis < 3; ++axis) {
    visualization_msgs::msg::Marker marker;
    marker.header = target.header;
    marker.ns = "box_target_axes";
    marker.id = axis;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.006;
    marker.scale.y = 0.014;
    marker.scale.z = 0.018;
    marker.color.r = colors[axis][0];
    marker.color.g = colors[axis][1];
    marker.color.b = colors[axis][2];
    marker.color.a = 1.0;
    geometry_msgs::msg::Point start = target.pose.position;
    geometry_msgs::msg::Point end = start;
    if (axis == 0) {end.x += 0.08;}
    if (axis == 1) {end.y += 0.08;}
    if (axis == 2) {end.z += 0.08;}
    marker.points = {start, end};
    array.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker point;
  point.header = target.header;
  point.ns = "box_target_point";
  point.id = 3;
  point.type = visualization_msgs::msg::Marker::SPHERE;
  point.action = visualization_msgs::msg::Marker::ADD;
  point.pose.position = target.pose.position;
  point.pose.orientation.w = 1.0;
  point.scale.x = point.scale.y = point.scale.z = 0.025;
  point.color.r = 1.0;
  point.color.g = 0.85;
  point.color.b = 0.0;
  point.color.a = 0.9;
  array.markers.push_back(point);

  visualization_msgs::msg::Marker text;
  text.header = target.header;
  text.ns = "box_target_text";
  text.id = 4;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position = target.pose.position;
  text.pose.position.z += 0.11;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.025;
  text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
  std::ostringstream label;
  label << std::fixed << std::setprecision(3) << "(" << target.pose.position.x << ", "
        << target.pose.position.y << ", " << target.pose.position.z << ")";
  text.text = label.str();
  array.markers.push_back(text);

  marker_publisher_->publish(array);
}

void OmxTargetPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  float value;
  if (config.mapGetFloat("X", &value)) {x_input_->setValue(value);}
  if (config.mapGetFloat("Y", &value)) {y_input_->setValue(value);}
  if (config.mapGetFloat("Z", &value)) {z_input_->setValue(value);}
}

void OmxTargetPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("X", x_input_->value());
  config.mapSetValue("Y", y_input_->value());
  config.mapSetValue("Z", z_input_->value());
}

}  // namespace omx_box_control

PLUGINLIB_EXPORT_CLASS(omx_box_control::OmxTargetPanel, rviz_common::Panel)
