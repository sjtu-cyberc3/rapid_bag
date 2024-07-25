
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

void read_binary(std::ifstream &ifs, rcl_serialized_message_t &msg) {
    std::size_t length = 0;
    ifs.read((char *)&length, sizeof(length));
    if (msg.buffer_capacity < length) {
        msg.buffer = (uint8_t *)msg.allocator.reallocate(msg.buffer, length, msg.allocator.state);
        msg.buffer_capacity = length;
    }
    msg.buffer_length = length;
    ifs.read((char *)msg.buffer, length);
}

void read_binary(std::ifstream &ifs, rcl_time_point_value_t &stamp) { ifs.read((char *)&stamp, sizeof(stamp)); }

void read_binary(std::ifstream &ifs, std::string &str) {
    std::size_t length = 0;
    ifs.read((char *)&length, sizeof(length));
    str.resize(length);
    ifs.read(str.data(), length);
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "usage: " << argv[0] << " <binary bag file> <output bag file>" << std::endl;
        return -1;
    }
    std::string bin_file = argv[1];
    std::string bag_file = argv[2];
    std::ifstream ifs(bin_file, std::ios_base::in | std::ios_base::binary);

    std::string bag_meta_data;
    read_binary(ifs, bag_meta_data);
    YAML::Node bag_meta = YAML::Load(bag_meta_data);

    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    writer->open({bag_file, "sqlite3"}, {rmw_get_serialization_format(), rmw_get_serialization_format()});

    for (auto topic_meta : bag_meta) {
        writer->create_topic({topic_meta["name"].as<std::string>(), topic_meta["type"].as<std::string>(),
                              rmw_get_serialization_format(), topic_meta["qos"].as<std::string>()});
    }

    while (1) {
        std::string topic_name, message_type;
        rcl_time_point_value_t message_stamp;
        auto p_msg = std::make_shared<rclcpp::SerializedMessage>();
        read_binary(ifs, topic_name);
        read_binary(ifs, message_type);
        read_binary(ifs, message_stamp);
        read_binary(ifs, p_msg->get_rcl_serialized_message());
        if (!ifs) break;
        writer->write(p_msg, topic_name, message_type, rclcpp::Time(message_stamp));
    }

    return 0;
}