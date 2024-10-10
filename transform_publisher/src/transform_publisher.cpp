#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class TransformPublisher
{
public:
  TransformPublisher(const std::string& parent_frame_id, const std::string& frame_id, const std::string& topic_to_publish, double check_duration) :
    buffer_(), tf2_listener_(buffer_), parent_frame_id_(parent_frame_id), frame_id_(frame_id), topic_to_publish_(topic_to_publish)
  {
    // Publisher to publish the TransformStamped messages
    transform_pub_ = n_.advertise<geometry_msgs::TransformStamped>(topic_to_publish_, 10);

    // Create a timer with the specified check_duration
    timer_ = n_.createTimer(ros::Duration(check_duration), &TransformPublisher::checkTransform, this);
  }

  // Function to check and publish the transform between parent_frame_id_ and frame_id_
  void checkTransform(const ros::TimerEvent&)
  {
    try
    {
      // Get the latest transform between parent_frame_id_ and frame_id_
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = buffer_.lookupTransform(parent_frame_id_, frame_id_, ros::Time(0));

      // Publish the TransformStamped message
      transform_pub_.publish(transform_stamped);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Could not get transform between %s and %s: %s", parent_frame_id_.c_str(), frame_id_.c_str(), ex.what());
    }
  }

private:
  std::string parent_frame_id_;
  std::string frame_id_;
  std::string topic_to_publish_;
  ros::NodeHandle n_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_listener_;
  ros::Publisher transform_pub_;  // Publisher to publish TransformStamped messages
  ros::Timer timer_;  // Timer to repeatedly check for the transform
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_publisher");

  // Check for the correct number of arguments
  if (argc != 5)
  {
    ROS_ERROR("Usage: transform_publisher <parent_frame_id> <frame_id> <topic_to_publish> <tf_check_duration>");
    return -1;
  }

  // Read arguments
  std::string parent_frame_id = argv[1];
  std::string frame_id = argv[2];
  std::string topic_to_publish = argv[3];
  double check_duration = std::stod(argv[4]);  // Convert the check_duration argument to a double

  // Construct the TransformPublisher object with the given arguments
  TransformPublisher tp(parent_frame_id, frame_id, topic_to_publish, check_duration);

  // Run the node
  ros::spin();

  return 0;
}
