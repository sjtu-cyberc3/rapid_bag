#include "rapid_bag/recorder.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/qos.hpp>

using namespace std::chrono_literals;

static std::string serialized_offered_qos_profiles(const std::vector<rclcpp::TopicEndpointInfo> &endpoints) {
    YAML::Node offered_qos_profiles;
    for (const auto &info : endpoints) {
        offered_qos_profiles.push_back(rosbag2_transport::Rosbag2QoS(info.qos_profile()));
    }
    return YAML::Dump(offered_qos_profiles);
}

void write_binary(std::ofstream &ofs, const rcl_serialized_message_t &msg) {
    std::size_t length = msg.buffer_length;
    ofs.write((const char *)&length, sizeof(length));
    ofs.write((const char *)msg.buffer, length);
}

void write_binary(std::ofstream &ofs, const rcl_time_point_value_t &stamp) {
    ofs.write((const char *)&stamp, sizeof(stamp));
}

void write_binary(std::ofstream &ofs, const std::string &str) {
    std::size_t length = str.size();
    ofs.write((const char *)&length, sizeof(length));
    ofs.write(str.data(), length);
}

std::string pretty_fps_counter(const std::map<std::string, int> &fps_counter) {
    std::size_t max_name_length = 0, max_fps_length = 0;
    for (auto &[name, fps] : fps_counter) {
        max_name_length = std::max(name.size(), max_name_length);
        max_fps_length = std::max(std::to_string(fps).size(), max_fps_length);
    }
    std::size_t cols = 2 + max_name_length + 3 + max_fps_length + 2 + 2;
    std::size_t rows = fps_counter.size() * 2 + 1;
    std::string buffer;
    buffer.resize(cols * rows + 1);

    auto it = fps_counter.begin();
    for (std::size_t r = 0; r < rows; r++) {
        if (r % 2 == 0) {
            char *ptr = &buffer[r * cols];
            *(ptr++) = '+';
            for (std::size_t i = 0; i < max_name_length + 2; i++) {
                *(ptr++) = '-';
            }
            *(ptr++) = '+';
            for (std::size_t i = 0; i < max_fps_length + 2; i++) {
                *(ptr++) = '-';
            }
            *(ptr++) = '+';
            *(ptr++) = '\r';
            *(ptr++) = '\n';
        } else {
            const auto &name = it->first;
            const auto &fps = std::to_string(it->second);
            char *ptr = &buffer[r * cols];
            *(ptr++) = '|';
            *(ptr++) = ' ';
            for (std::size_t i = 0; i < max_name_length; i++) {
                *(ptr++) = i < name.size() ? name[i] : ' ';
            }
            *(ptr++) = ' ';
            *(ptr++) = '|';
            *(ptr++) = ' ';
            for (std::size_t i = 0; i < max_fps_length; i++) {
                *(ptr++) = i < fps.size() ? fps[i] : ' ';
            }
            *(ptr++) = ' ';
            *(ptr++) = '|';
            *(ptr++) = '\r';
            *(ptr++) = '\n';
            it++;
        }
    }

    return buffer;
}

namespace rapid_bag {

struct Recorder::Impl {
    std::vector<std::string> topics;
    std::string output_path;
    bool dummy = false;
    bool quiet = false;

    std::ofstream ofs;

    std::chrono::time_point<std::chrono::steady_clock> fps_timer;
    std::map<std::string, int> fps_counter;

    std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscribers;
};

Recorder::Recorder(const rclcpp::NodeOptions &options)
    : rclcpp::Node("rapid_bag_recorder_node", options), pImpl(std::make_unique<Impl>()) {
    declare_parameter<std::string>("output");
    pImpl->output_path = get_parameter("output").as_string();

    declare_parameter<std::vector<std::string>>("topics");
    pImpl->topics = get_parameter("topics").as_string_array();

    declare_parameter<bool>("dummy", false);
    pImpl->dummy = get_parameter("dummy").as_bool();

    declare_parameter<bool>("quiet", false);
    pImpl->quiet = get_parameter("quiet").as_bool();

    YAML::Node bag_meta;
    for (size_t i = 0; i < pImpl->topics.size(); i++) {
        const std::string topic_name = pImpl->topics[i];
        auto endpoints = get_publishers_info_by_topic(topic_name);
        if (endpoints.empty()) {
            RCLCPP_WARN(get_logger(), "topic %s has no publisher! skip recording!", topic_name.c_str());
            continue;
        }
        const std::string message_type = endpoints[0].topic_type();
        YAML::Node msg_meta;
        msg_meta["name"] = topic_name;
        msg_meta["type"] = message_type;
        msg_meta["qos"] = serialized_offered_qos_profiles(endpoints);
        bag_meta.push_back(msg_meta);

        auto qos = rosbag2_transport::Rosbag2QoS::adapt_request_to_offers(topic_name, endpoints).keep_last(1000);
        auto generic_callback = [this, topic_name, message_type](std::shared_ptr<rclcpp::SerializedMessage> p_msg) {
            auto message_stamp = get_clock()->now();
            if (!pImpl->quiet) {
                auto fps_stamp = std::chrono::steady_clock::now();
                pImpl->fps_counter[topic_name]++;
                if (fps_stamp - pImpl->fps_timer >= 1s) {
                    pImpl->fps_timer = fps_stamp;
                    RCLCPP_INFO(get_logger(), "\033[2J\033[1;1H\n%s", pretty_fps_counter(pImpl->fps_counter).c_str());
                    for (auto &[_, counter] : pImpl->fps_counter) {
                        counter = 0;
                    }
                }
            }
            if (!pImpl->dummy) {
                write_binary(pImpl->ofs, topic_name);
                write_binary(pImpl->ofs, message_type);
                write_binary(pImpl->ofs, message_stamp.nanoseconds());
                write_binary(pImpl->ofs, p_msg->get_rcl_serialized_message());
            }
        };
        pImpl->subscribers[topic_name] = create_generic_subscription(topic_name, message_type, qos, generic_callback);
        pImpl->fps_counter[topic_name] = 0;
    }
    pImpl->fps_timer = std::chrono::steady_clock::now();

    if (!pImpl->dummy) {
        pImpl->ofs.open(pImpl->output_path, std::ios_base::out | std::ios_base::binary);
        std::string bag_meta_data = YAML::Dump(bag_meta);
        write_binary(pImpl->ofs, bag_meta_data);
    }
}

Recorder::~Recorder() = default;

}  // namespace rapid_bag

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rapid_bag::Recorder)