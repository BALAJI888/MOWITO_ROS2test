#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "nodes.h"

int main()
{
    BT::BehaviorTreeFactory factory;
    RegisterNodes(factory);
    
    // Define the behavior tree XML
    std::string xml_tree = R"(
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <!-- Navigate to room and enter -->
                <MoveTowardsDoorOfRoom name="move_to_room_door"/>
                
                <Fallback name="room_door_handling">
                    <IsDoorClosed door_type="room"/>
                    <AlwaysSuccess/>
                </Fallback>
                
                <Sequence name="open_room_door_if_closed">
                    <Fallback name="check_and_open_room">
                        <IsDoorClosed door_type="room"/>
                        <AlwaysSuccess/>
                    </Fallback>
                    <OpenDoor name="open_room_door"/>
                </Sequence>
                
                <EnterRoom name="enter_room"/>
                
                <!-- Navigate to fridge and get apple -->
                <MoveTowardsDoorOfFridge name="move_to_fridge_door"/>
                
                <Fallback name="fridge_door_handling">
                    <IsDoorClosed door_type="fridge"/>
                    <AlwaysSuccess/>
                </Fallback>
                
                <Sequence name="open_fridge_door_if_closed">
                    <Fallback name="check_and_open_fridge">
                        <IsDoorClosed door_type="fridge"/>
                        <AlwaysSuccess/>
                    </Fallback>
                    <OpenDoor name="open_fridge_door"/>
                </Sequence>
                
                <FindApple name="find_apple"/>
                <PickApple name="pick_apple"/>
                
                <!-- Clean up and exit -->
                <CloseDoorOfFridge name="close_fridge_door"/>
                <MoveTowardsDoorOfRoom name="move_back_to_room_door"/>
                <ExitRoom name="exit_room"/>
            </Sequence>
        </BehaviorTree>
    </root>
    )";
    
    try {
        // Create tree from XML
        auto tree = factory.createTreeFromText(xml_tree);
        
        // Add logger to see what's happening
        BT::StdCoutLogger logger(tree);
        
        std::cout << "=== Starting Behavior Tree Execution ===" << std::endl;
        
        // Execute the tree
        tree.tickWhileRunning();
        
        std::cout << "=== Behavior Tree Execution Completed ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}