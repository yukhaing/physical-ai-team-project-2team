#include "omx_target_control/omx_target_panel.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace omx_target_control
{

OmxTargetPanel::OmxTargetPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * form = new QFormLayout();
  x_input_ = makeInput(0.20, 0.08, 0.32);
  y_input_ = makeInput(0.00, -0.25, 0.25);
  z_input_ = makeInput(0.15, 0.01, 0.32);
  form->addRow("X (m)", x_input_);
  form->addRow("Y (m)", y_input_);
  form->addRow("Z (m)", z_input_);
  move_button_ = new QPushButton("Move");
  workspace_button_ = new QPushButton("Show Reachable Workspace");
  workspace_button_->setCheckable(true);
  workspace_button_->setChecked(true);
  status_label_ = new QLabel("Waiting for RViz initialization");
  status_label_->setWordWrap(true);
  move_button_->setEnabled(false);
  workspace_button_->setEnabled(false);
  auto * layout = new QVBoxLayout();
  layout->addLayout(form);
  layout->addWidget(move_button_);
  layout->addWidget(workspace_button_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);
  connect(move_button_, &QPushButton::clicked, this, &OmxTargetPanel::publishTarget);
  connect(workspace_button_, &QPushButton::toggled, this, &OmxTargetPanel::toggleWorkspace);
}

QDoubleSpinBox * OmxTargetPanel::makeInput(double value, double minimum, double maximum)
{
  auto * input = new QDoubleSpinBox();
  input->setDecimals(3);
  input->setSingleStep(0.01);
  input->setRange(minimum, maximum);
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
  target_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/box_target_pose", 10);
  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/box_target_marker", rclcpp::QoS(1).transient_local().reliable());
  move_button_->setEnabled(true);
  workspace_button_->setEnabled(true);
  toggleWorkspace(true);
  status_label_->setText("Ready: /box_target_pose (frame: link0)");
}

void OmxTargetPanel::publishTarget()
{
  const double x = x_input_->value();
  const double y = y_input_->value();
  const double z = z_input_->value();
  const double reach = std::sqrt(x * x + y * y + z * z);
  if (!target_publisher_ || reach < 0.10 || reach > 0.42) {
    status_label_->setText("Rejected: target is outside radial reach");
    return;
  }
  geometry_msgs::msg::PoseStamped target;
  target.header.stamp = node_->now();
  target.header.frame_id = "link0";
  target.pose.position.x = x;
  target.pose.position.y = y;
  target.pose.position.z = z;
  target.pose.orientation.w = 1.0;
  target_publisher_->publish(target);
  publishTargetMarkers(target);
  status_label_->setText(
    QString("Sent: x=%1, y=%2, z=%3").arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
}

void OmxTargetPanel::toggleWorkspace(bool visible)
{
  if (!marker_publisher_) {return;}
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "link0";
  marker.header.stamp = node_->now();
  marker.ns = "omx_reachable_workspace";
  marker.id = 10;
  if (!visible) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(marker);
    marker_publisher_->publish(array);
    workspace_button_->setText("Show Reachable Workspace");
    return;
  }
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = 0.012;
  marker.color.r = 0.1F;
  marker.color.g = 1.0F;
  marker.color.b = 0.25F;
  marker.color.a = 0.22F;
  for (double x = 0.08; x <= 0.32; x += 0.02) {
    for (double y = -0.24; y <= 0.24; y += 0.02) {
      for (double z = 0.02; z <= 0.32; z += 0.02) {
        const double reach = std::sqrt(x * x + y * y + z * z);
        if (reach < 0.10 || reach > 0.42) {continue;}
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        marker.points.push_back(point);
      }
    }
  }
  array.markers.push_back(marker);
  marker_publisher_->publish(array);
  workspace_button_->setText("Hide Reachable Workspace");
}

void OmxTargetPanel::publishTargetMarkers(const geometry_msgs::msg::PoseStamped & target)
{
  visualization_msgs::msg::MarkerArray array;
  const std::array<std::array<float, 3>, 3> colors = {{
    {{1.0F, 0.0F, 0.0F}}, {{0.0F, 1.0F, 0.0F}}, {{0.0F, 0.4F, 1.0F}}
  }};
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
  visualization_msgs::msg::Marker label;
  label.header = target.header;
  label.ns = "box_target_label";
  label.id = 4;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.position = target.pose.position;
  label.pose.position.z += 0.10;
  label.pose.orientation.w = 1.0;
  label.scale.z = 0.025;
  label.color.r = label.color.g = label.color.b = label.color.a = 1.0;
  std::ostringstream text;
  text << std::fixed << std::setprecision(3) << "(" << target.pose.position.x << ", "
       << target.pose.position.y << ", " << target.pose.position.z << ")";
  label.text = text.str();
  array.markers.push_back(label);
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

}  // namespace omx_target_control

PLUGINLIB_EXPORT_CLASS(omx_target_control::OmxTargetPanel, rviz_common::Panel)
