#pragma once
#ifndef RAPID_BAG_RECORDER_HPP
#define RAPID_BAG_RECORDER_HPP

#include <rclcpp/rclcpp.hpp>

namespace rapid_bag {

class Recorder : public rclcpp::Node {
   public:
    Recorder(const rclcpp::NodeOptions &options);
    ~Recorder();

   private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

}  // namespace rapid_bag

#endif  // RAPID_BAG_RECORDER_HPP