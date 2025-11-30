#include "nodes.h"
#include <iostream>

BT::NodeStatus MoveTowardsDoorOfRoom::tick()
{
    std::cout << "[MoveTowardsDoorOfRoom] Moving towards the door of the room..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OpenDoor::tick()
{
    std::cout << "[OpenDoor] Opening the door..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus EnterRoom::tick()
{
    std::cout << "[EnterRoom] Entering the room..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveTowardsDoorOfFridge::tick()
{
    std::cout << "[MoveTowardsDoorOfFridge] Moving towards the door of the fridge..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindApple::tick()
{
    std::cout << "[FindApple] Finding the apple in the fridge..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PickApple::tick()
{
    std::cout << "[PickApple] Picking the apple..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CloseDoorOfFridge::tick()
{
    std::cout << "[CloseDoorOfFridge] Closing the door of the fridge..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExitRoom::tick()
{
    std::cout << "[ExitRoom] Exiting the room..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsDoorClosed::tick()
{
    auto door_type = getInput<std::string>("door_type");
    if (!door_type) {
        throw BT::RuntimeError("Missing parameter [door_type] in IsDoorClosed");
    }

    // For demonstration, let's assume doors are initially closed
    bool is_closed = true;
    
    if (door_type.value() == "room") {
        std::cout << "[IsDoorClosed] Checking if room door is closed: " 
                  << (is_closed ? "YES" : "NO") << std::endl;
    } else if (door_type.value() == "fridge") {
        std::cout << "[IsDoorClosed] Checking if fridge door is closed: " 
                  << (is_closed ? "YES" : "NO") << std::endl;
    }

    return is_closed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    // Register action nodes
    factory.registerNodeType<MoveTowardsDoorOfRoom>("MoveTowardsDoorOfRoom");
    factory.registerNodeType<OpenDoor>("OpenDoor");
    factory.registerNodeType<EnterRoom>("EnterRoom");
    factory.registerNodeType<MoveTowardsDoorOfFridge>("MoveTowardsDoorOfFridge");
    factory.registerNodeType<FindApple>("FindApple");
    factory.registerNodeType<PickApple>("PickApple");
    factory.registerNodeType<CloseDoorOfFridge>("CloseDoorOfFridge");
    factory.registerNodeType<ExitRoom>("ExitRoom");
    
    // Register condition nodes
    factory.registerNodeType<IsDoorClosed>("IsDoorClosed");
}