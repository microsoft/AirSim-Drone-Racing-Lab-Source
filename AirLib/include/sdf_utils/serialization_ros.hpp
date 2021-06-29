#ifndef SERIALIZATION_ROS_HPP
#define SERIALIZATION_ROS_HPP

#include "sdf_utils/serialization.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace sdf_utils
{
    inline uint64_t SerializeHeader(
            const std_msgs::Header& header,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeFixedSizePOD(header.seq, buffer);
        bytes_written += SerializeFixedSizePOD(header.stamp.sec, buffer);
        bytes_written += SerializeFixedSizePOD(header.stamp.nsec, buffer);
        bytes_written += SerializeString(header.frame_id, buffer);
        return bytes_written;
    }

    inline std::pair<std_msgs::Header, uint64_t> DeserializeHeader(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        std_msgs::Header header;
        uint64_t bytes_read = 0;
        const auto deserialized_seq = DeserializeFixedSizePOD<decltype(header.seq)>(buffer, current + bytes_read);
        header.seq = deserialized_seq.first;
        bytes_read += deserialized_seq.second;
        const auto deserialized_sec = DeserializeFixedSizePOD<decltype(header.stamp.sec)>(buffer, current + bytes_read);
        header.stamp.sec = deserialized_sec.first;
        bytes_read += deserialized_sec.second;
        const auto deserialized_nsec = DeserializeFixedSizePOD<decltype(header.stamp.sec)>(buffer, current + bytes_read);
        header.stamp.nsec = deserialized_nsec.first;
        bytes_read += deserialized_nsec.second;
        const auto deserialized_frame_id = DeserializeString<char>(buffer, current + bytes_read);
        header.frame_id = deserialized_frame_id.first;
        bytes_read += deserialized_frame_id.second;
        return {header, bytes_read};
    }

    inline uint64_t SerializePoint(
            const geometry_msgs::Point& point,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeFixedSizePOD(point.x, buffer);
        bytes_written += SerializeFixedSizePOD(point.y, buffer);
        bytes_written += SerializeFixedSizePOD(point.z, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::Point, uint64_t> DeserializePoint(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::Point point;
        uint64_t bytes_read = 0;
        const auto deserialized_x = DeserializeFixedSizePOD<decltype(point.x)>(buffer, current + bytes_read);
        point.x = deserialized_x.first;
        bytes_read += deserialized_x.second;
        const auto deserialized_y = DeserializeFixedSizePOD<decltype(point.y)>(buffer, current + bytes_read);
        point.y = deserialized_y.first;
        bytes_read += deserialized_y.second;
        const auto deserialized_z = DeserializeFixedSizePOD<decltype(point.z)>(buffer, current + bytes_read);
        point.z = deserialized_z.first;
        bytes_read += deserialized_z.second;
        return {point, bytes_read};
    }

    inline uint64_t SerializeVector3(
            const geometry_msgs::Vector3& vector,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeFixedSizePOD(vector.x, buffer);
        bytes_written += SerializeFixedSizePOD(vector.y, buffer);
        bytes_written += SerializeFixedSizePOD(vector.z, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::Vector3, uint64_t> DeserializeVector3(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::Vector3 vector;
        uint64_t bytes_read = 0;
        const auto deserialized_x = DeserializeFixedSizePOD<decltype(vector.x)>(buffer, current + bytes_read);
        vector.x = deserialized_x.first;
        bytes_read += deserialized_x.second;
        const auto deserialized_y = DeserializeFixedSizePOD<decltype(vector.y)>(buffer, current + bytes_read);
        vector.y = deserialized_y.first;
        bytes_read += deserialized_y.second;
        const auto deserialized_z = DeserializeFixedSizePOD<decltype(vector.z)>(buffer, current + bytes_read);
        vector.z = deserialized_z.first;
        bytes_read += deserialized_z.second;
        return {vector, bytes_read};
    }

    inline uint64_t SerializeQuaternion(
            const geometry_msgs::Quaternion& quat,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeFixedSizePOD(quat.x, buffer);
        bytes_written += SerializeFixedSizePOD(quat.y, buffer);
        bytes_written += SerializeFixedSizePOD(quat.z, buffer);
        bytes_written += SerializeFixedSizePOD(quat.w, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::Quaternion, uint64_t> DeserializeQuaternion(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::Quaternion quat;
        uint64_t bytes_read = 0;
        const auto deserialized_x = DeserializeFixedSizePOD<decltype(quat.x)>(buffer, current + bytes_read);
        quat.x = deserialized_x.first;
        bytes_read += deserialized_x.second;
        const auto deserialized_y = DeserializeFixedSizePOD<decltype(quat.y)>(buffer, current + bytes_read);
        quat.y = deserialized_y.first;
        bytes_read += deserialized_y.second;
        const auto deserialized_z = DeserializeFixedSizePOD<decltype(quat.z)>(buffer, current + bytes_read);
        quat.z = deserialized_z.first;
        bytes_read += deserialized_z.second;
        const auto deserialized_w = DeserializeFixedSizePOD<decltype(quat.z)>(buffer, current + bytes_read);
        quat.w = deserialized_w.first;
        bytes_read += deserialized_w.second;
        return {quat, bytes_read};
    }

    inline uint64_t SerializePose(
            const geometry_msgs::Pose& pose,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializePoint(pose.position, buffer);
        bytes_written += SerializeQuaternion(pose.orientation, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::Pose, uint64_t> DeserializePose(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::Pose pose;
        uint64_t bytes_read = 0;
        const auto deserialized_position = DeserializePoint(buffer, current + bytes_read);
        pose.position = deserialized_position.first;
        bytes_read += deserialized_position.second;
        const auto deserialized_quat = DeserializeQuaternion(buffer, current + bytes_read);
        pose.orientation = deserialized_quat.first;
        bytes_read += deserialized_quat.second;
        return {pose, bytes_read};
    }

    inline uint64_t SerializePoseStamped(
            const geometry_msgs::PoseStamped& pose,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeHeader(pose.header, buffer);
        bytes_written += SerializePose(pose.pose, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::PoseStamped, uint64_t> DeserializePoseStamped(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::PoseStamped pose;
        uint64_t bytes_read = 0;
        const auto deserialized_header = DeserializeHeader(buffer, current + bytes_read);
        pose.header = deserialized_header.first;
        bytes_read += deserialized_header.second;
        const auto deserialized_pose = DeserializePose(buffer, current + bytes_read);
        pose.pose = deserialized_pose.first;
        bytes_read += deserialized_pose.second;
        return {pose, bytes_read};
    }

    inline uint64_t SerializeTransform(
            const geometry_msgs::Transform& transform,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeVector3(transform.translation, buffer);
        bytes_written += SerializeQuaternion(transform.rotation, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::Transform, uint64_t> DeserializeTransform(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::Transform transform;
        uint64_t bytes_read = 0;
        const auto deserialized_translation = DeserializeVector3(buffer, current + bytes_read);
        transform.translation = deserialized_translation.first;
        bytes_read += deserialized_translation.second;
        const auto deserialized_quat = DeserializeQuaternion(buffer, current + bytes_read);
        transform.rotation = deserialized_quat.first;
        bytes_read += deserialized_quat.second;
        return {transform, bytes_read};
    }

    inline uint64_t SerializeTransformStamped(
            const geometry_msgs::TransformStamped& transform,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        bytes_written += SerializeHeader(transform.header, buffer);
        bytes_written += SerializeTransform(transform.transform, buffer);
        bytes_written += SerializeString<char>(transform.child_frame_id, buffer);
        return bytes_written;
    }

    inline std::pair<geometry_msgs::TransformStamped, uint64_t> DeserializeTransformStamped(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        geometry_msgs::TransformStamped transform;
        uint64_t bytes_read = 0;
        const auto deserialized_header = DeserializeHeader(buffer, current + bytes_read);
        transform.header = deserialized_header.first;
        bytes_read += deserialized_header.second;
        const auto deserialized_transform = DeserializeTransform(buffer, current + bytes_read);
        transform.transform = deserialized_transform.first;
        bytes_read += deserialized_transform.second;
        const auto deserialized_child_frame_id = DeserializeString<char>(buffer, current + bytes_read);
        transform.child_frame_id = deserialized_child_frame_id.first;
        bytes_read += deserialized_child_frame_id.second;
        return {transform, bytes_read};
    }

    template <typename MessageType>
    inline uint64_t RosMessageSerializationWrapper(
            const MessageType& message,
            std::vector<uint8_t>& buffer)
    {
        const ros::SerializedMessage serialized_message = ros::serialization::serializeMessage(message);
        buffer.insert(buffer.end(), serialized_message.message_start, serialized_message.message_start + serialized_message.num_bytes);
        return serialized_message.num_bytes;
    }

    template <typename MessageType>
    inline std::pair<MessageType, uint64_t> RosMessageDeserializationWrapper(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        // Convert the input buffer into the ROS/Boost format
        const auto size_left = buffer.size() - current;
        boost::shared_array<uint8_t> boost_buffer(new uint8_t[size_left]);
        std::memcpy(boost_buffer.get(), &buffer[current], size_left);
        const ros::SerializedMessage serialize_message(boost_buffer, size_left);

        // Perform the actual deserialization
        MessageType message;
        ros::serialization::deserializeMessage(serialize_message, message);
        // +4 is here to account for what happens in serializeMessage(...)
        // This is used by serializeMessage to store the total size of the message in the buffer (excluding this length marker)
        const uint64_t bytes_read = ros::serialization::serializationLength(message) + 4;
        return {message, bytes_read};
    }
}

#endif // SERIALIZATION_ROS_HPP
