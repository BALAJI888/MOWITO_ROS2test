#ifndef NODES_H
#define NODES_H

#include "behaviortree_cpp/bt_factory.h"

// Action Nodes
class MoveTowardsDoorOfRoom : public BT::SyncActionNode
{
public:
    MoveTowardsDoorOfRoom(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class OpenDoor : public BT::SyncActionNode
{
public:
    OpenDoor(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class EnterRoom : public BT::SyncActionNode
{
public:
    EnterRoom(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class MoveTowardsDoorOfFridge : public BT::SyncActionNode
{
public:
    MoveTowardsDoorOfFridge(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class FindApple : public BT::SyncActionNode
{
public:
    FindApple(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class PickApple : public BT::SyncActionNode
{
public:
    PickApple(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class CloseDoorOfFridge : public BT::SyncActionNode
{
public:
    CloseDoorOfFridge(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

class ExitRoom : public BT::SyncActionNode
{
public:
    ExitRoom(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override;
};

// Condition Nodes
class IsDoorClosed : public BT::ConditionNode
{
public:
    IsDoorClosed(const std::string& name, const BT::NodeConfig& config) 
        : BT::ConditionNode(name, config) {}
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("door_type") };
    }
    
    BT::NodeStatus tick() override;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

#endif
