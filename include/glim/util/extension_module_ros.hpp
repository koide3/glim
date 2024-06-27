#pragma once

#include <spdlog/spdlog.h>
#include <ros/ros.h>
#include <rosbag/message_instance.h>
#include <glim/util/extension_module.hpp>

namespace glim {

/**
 * @brief Generic topic subscription that allows transparently subscribes to a topic through glim_rosnode and glim_rosbag
 */
class GenericTopicSubscription {
public:
  using Ptr = std::shared_ptr<GenericTopicSubscription>;

  /**
   * @brief Constructor
   * @param topic Topic name
   */
  GenericTopicSubscription(const std::string& topic) : topic(topic) {}

  /**
   * @brief Create a ROS subscriber
   * @param nh Node handle
   */
  virtual void create_subscriber(ros::NodeHandle& nh) = 0;

  /**
   * @brief Parse a rosbag message instance and feed the msg to the callback
   * @param m  Message instance
   */
  virtual void insert_message_instance(const rosbag::MessageInstance& m) = 0;

  /// Topic name
  const std::string topic;
};

/**
 * @brief Specialized topic subscription for type erasure
 */
template <typename Msg>
class TopicSubscription : public GenericTopicSubscription {
public:
  /**
   * @brief topic    Topic name
   * @brief callback Message callback
   */
  template <typename Callback>
  TopicSubscription(const std::string& topic, const Callback& callback) : GenericTopicSubscription(topic),
                                                                          callback(callback) {}

  virtual void create_subscriber(ros::NodeHandle& nh) override { sub = nh.subscribe<Msg>(topic, 10, callback); }

  virtual void insert_message_instance(const rosbag::MessageInstance& m) override {
    const auto msg = m.instantiate<Msg>();
    if (msg == nullptr) {
      spdlog::warn("failed to instantiate message on {}", topic);
      return;
    }

    callback(msg);
  }

  const std::function<void(const boost::shared_ptr<const Msg>&)> callback;
  ros::Subscriber sub;
};

/**
 * @brief Extension module with ROS1 topic subscription
 */
class ExtensionModuleROS : public ExtensionModule {
public:
  ExtensionModuleROS() {}
  virtual ~ExtensionModuleROS() {}

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() = 0;
};

}  // namespace glim