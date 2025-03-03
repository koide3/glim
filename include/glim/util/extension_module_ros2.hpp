#pragma once

#include <any>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class GenericTopicSubscription {
public:
  using Ptr = std::shared_ptr<GenericTopicSubscription>;

  GenericTopicSubscription(const std::string& topic, const std::string& msg_type = "") : topic(topic), msg_type(msg_type) {}

  virtual void create_subscriber(rclcpp::Node& node) = 0;
  virtual void insert_message_instance(const rclcpp::SerializedMessage& serialized_msg, const std::string& msg_type = "") = 0;

  const std::string topic;
  const std::string msg_type;
};

template <typename Msg>
class TopicSubscription : public GenericTopicSubscription {
public:
  template <typename Callback>
  TopicSubscription(const std::string& topic, const Callback& callback) : GenericTopicSubscription(topic),
                                                                          callback(callback) {}
  template <typename Callback>
  TopicSubscription(const std::string& topic, const std::string& msg_type, const Callback& callback) : GenericTopicSubscription(topic, msg_type),
                                                                                                       callback(callback) {}
  ~TopicSubscription() {}

  virtual void create_subscriber(rclcpp::Node& node) override {
    if (!this->msg_type.empty()) {
      const auto topics_types = node.get_topic_names_and_types();
      const auto found = topics_types.find(topic);
      if (found != topics_types.end()) {
        for (const auto& type : found->second) {
          if (type != this->msg_type) {
            spdlog::warn("msg type mismatch: topic={} expected={} actual={}", topic, this->msg_type, type);
          }
        }
      }
    }

    sub = node.create_subscription<Msg>(topic, 100, [&](const std::shared_ptr<Msg> msg) { callback(msg); });
  }

  virtual void insert_message_instance(const rclcpp::SerializedMessage& serialized_msg, const std::string& msg_type = "") override {
    if (!msg_type.empty() && !this->msg_type.empty() && msg_type != this->msg_type) {
      spdlog::warn("msg type mismatch: topic={} expected={} actual={}", topic, this->msg_type, msg_type);
      return;
    }

    auto msg = std::make_shared<Msg>();
    serialization.deserialize_message(&serialized_msg, msg.get());

    if (msg == nullptr) {
      spdlog::warn("failed to deserialize message on {}", topic);
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

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) { return create_subscriptions(); }
  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() { return {}; }
};

}  // namespace glim