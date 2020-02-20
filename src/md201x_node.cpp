/*
 * md201x_node.cpp
 *
 *  Created on: Feb 19, 2020
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

#define CAN_MTU 8

template <typename T>
union _Encapsulator {
    T data;
    uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
    _Encapsulator<T> _e;

    for (int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }

    data = _e.data;
}

template <typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for (int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

class Md201xNode
{
public:
    Md201xNode(void);

private:
    void motorCmdCallback(const std_msgs::UInt8::ConstPtr &msg);
    void motorCmdValCallback(const std_msgs::Float32::ConstPtr &msg);

    void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    template <typename T>
    void sendData(const uint16_t id, const T data);

    ros::NodeHandle _nh;
    ros::Publisher _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Publisher _motor_status_pub;
    ros::Subscriber _motor_cmd_sub;
    ros::Subscriber _motor_cmd_val_sub;

    std::string bid;
    uint16_t id_motor_cmd;
    uint16_t id_motor_cmd_val;
    uint16_t id_motor_status;

    std::string name;
};

Md201xNode::Md201xNode(void)
{
    auto private_nh = ros::NodeHandle("~");
    private_nh.getParam("bid", this->bid);
    private_nh.getParam("name", this->name);

    this->id_motor_cmd = std::strtol(this->bid.c_str(), NULL, 16);
    this->id_motor_cmd_val = this->id_motor_cmd + 1;
    this->id_motor_status = this->id_motor_cmd + 3;
    
    _can_tx_pub = _nh.advertise<can_msgs::CanFrame>("can_tx", 10);
    _can_rx_sub = _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &Md201xNode::canRxCallback, this);

    _motor_status_pub = _nh.advertise<std_msgs::UInt8>(name + "_status", 10);
    _motor_cmd_sub = _nh.subscribe<std_msgs::UInt8>(name + "_cmd", 10, &Md201xNode::motorCmdCallback, this);
    _motor_cmd_val_sub = _nh.subscribe<std_msgs::Float32>(name + "_cmd_val", 10, &Md201xNode::motorCmdValCallback, this);
}

void Md201xNode::motorCmdCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    this->sendData(id_motor_cmd, msg->data);
}

void Md201xNode::motorCmdValCallback(const std_msgs::Float32::ConstPtr &msg)
{
    this->sendData(id_motor_cmd_val, msg->data);
}

void Md201xNode::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
    if (msg->id == id_motor_status)
    {
        std_msgs::UInt8 _motor_status_msg;
        can_unpack(msg->data, _motor_status_msg.data);
        _motor_status_pub.publish(_motor_status_msg);
    }
}

template <typename T>
void Md201xNode::sendData(const uint16_t id, const T data)
{
    can_msgs::CanFrame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);

    can_pack<T>(frame.data, data);

    _can_tx_pub.publish(frame);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "md201x_node");
    ROS_INFO("md201x node has started.");

    Md201xNode *md201xNode = new Md201xNode();

    ros::spin();
}