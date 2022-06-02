#pragma once

#include <rclcpp/rclcpp.hpp>
#include <glim/util/console_colors.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class GenericTopicSubscription {
public:
  using Ptr = std::shared_ptr<GenericTopicSubscription>;

  GenericTopicSubscription(const std::string& topic) : topic(topic) {}

  virtual void create_subscriber() = 0;
  virtual void insert_message_instance() = 0;

  const std::string topic;
};

template<typename Msg>
class TopicSubscription : public GenericTopicSubscription {
public:
  template<typename Callback>
  TopicSubscription(const std::string& topic, const Callback& callback) : GenericTopicSubscription(topic), callback(callback) {}

  virtual void create_subscriber() override {
  }

  virtual void insert_message_instance() override {
    /*
    const auto msg = m.instantiate<Msg>();
    if(msg == nullptr) {
      std::cerr << console::yellow << "warning: failed to instantiate message on " << topic << console::reset << std::endl;
      return;
    }

    callback(msg);
    */
  }

  const std::function<void(const boost::shared_ptr<const Msg>&)> callback;
  // ros::Subscriber sub;
};

class ExtensionModuleROS2 : public ExtensionModule {
public: 
  ExtensionModuleROS2() {}
  virtual ~ExtensionModuleROS2() {}

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() = 0;
};

}