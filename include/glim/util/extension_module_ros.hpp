#pragma once

#include <ros/ros.h>
#include <rosbag/message_instance.h>
#include <glim/util/console_colors.hpp>
#include <glim/util/extension_module.hpp>

namespace glim {

class GenericTopicSubscription {
public:
  using Ptr = std::shared_ptr<GenericTopicSubscription>;

  GenericTopicSubscription(const std::string& topic) : topic(topic) {}

  virtual void create_subscriber(ros::NodeHandle& nh) = 0;
  virtual void insert_message_instance(const rosbag::MessageInstance& m) = 0;

  const std::string topic;
};

template<typename Msg>
class TopicSubscription : public GenericTopicSubscription {
public:
  template<typename Callback>
  TopicSubscription(const std::string& topic, const Callback& callback) : GenericTopicSubscription(topic), callback(callback) {}

  virtual void create_subscriber(ros::NodeHandle& nh) override {
    sub = nh.subscribe<Msg>(topic, 10, callback);
  }

  virtual void insert_message_instance(const rosbag::MessageInstance& m) override {
    const auto msg = m.instantiate<Msg>();
    if(msg == nullptr) {
      std::cerr << console::yellow << "warning: failed to instantiate message on " << topic << console::reset << std::endl;
      return;
    }

    callback(msg);
  }

  const std::function<void(const boost::shared_ptr<const Msg>&)> callback;
  ros::Subscriber sub;
};

class ExtensionModuleROS : public ExtensionModule {
public: 
  ExtensionModuleROS() {}
  virtual ~ExtensionModuleROS() {}

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() = 0;
};

}