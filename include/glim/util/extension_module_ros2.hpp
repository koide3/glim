#pragma once

#include <any>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class GenericTopicSubscription {
public:
  using Ptr = std::shared_ptr<GenericTopicSubscription>;

  GenericTopicSubscription(const std::string& topic) : topic(topic) {}

  virtual void create_subscriber(rclcpp::Node& node) = 0;
  virtual void insert_message_instance(const rclcpp::SerializedMessage& serialized_msg) = 0;

  const std::string topic;
};

template <typename Msg>
class TopicSubscription : public GenericTopicSubscription {
public:
  template <typename Callback>
  TopicSubscription(const std::string& topic, const Callback& callback) : GenericTopicSubscription(topic),
                                                                          callback(callback) {}
  ~TopicSubscription() {}

  virtual void create_subscriber(rclcpp::Node& node) override {
    sub = node.create_subscription<Msg>(topic, 100, [&](const std::shared_ptr<Msg> msg) { callback(msg); });
  }

  virtual void insert_message_instance(const rclcpp::SerializedMessage& serialized_msg) override {
    auto msg = std::make_shared<Msg>();
    serialization.deserialize_message(&serialized_msg, msg.get());

    if (msg == nullptr) {
      std::cerr << console::yellow << "warning: failed to deserialize message on " << topic << console::reset << std::endl;
      return;
    }

    callback(msg);
  }

  const std::function<void(const std::shared_ptr<const Msg>&)> callback;
  rclcpp::Serialization<Msg> serialization;
  std::shared_ptr<rclcpp::Subscription<Msg>> sub;
};

class ExtensionModuleROS2 : public ExtensionModule {
public:
  ExtensionModuleROS2() {}
  virtual ~ExtensionModuleROS2() {}

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() = 0;
};

}  // namespace glim