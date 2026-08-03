#ifndef OMX_BOX_CONTROL__OMX_TARGET_PANEL_HPP_
#define OMX_BOX_CONTROL__OMX_TARGET_PANEL_HPP_

#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace omx_box_control
{

class OmxTargetPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit OmxTargetPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void publishTarget();
  void toggleWorkspace(bool visible);
  void publishTargetMarkers(const geometry_msgs::msg::PoseStamped & target);

private:
  QDoubleSpinBox * makePositionInput(double value);
  QDoubleSpinBox * x_input_;
  QDoubleSpinBox * y_input_;
  QDoubleSpinBox * z_input_;
  QLabel * status_label_;
  QPushButton * move_button_;
  QPushButton * workspace_button_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace omx_box_control

#endif  // OMX_BOX_CONTROL__OMX_TARGET_PANEL_HPP_
