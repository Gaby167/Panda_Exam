#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <iostream>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = rclcpp::Node::make_shared("panda_inverse_kinematics", node_options);

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(20.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    // Añadir obstáculos

    // Objeto 1:
    moveit_msgs::msg::CollisionObject obstacle1;
    obstacle1.id = "obstacle1";
    obstacle1.header.frame_id = "panda_link0";

    shape_msgs::msg::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions = {0.1, 0.1, 0.1};  // tamaño del cubo

    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.1;
    pose1.position.y = 0.4;
    pose1.position.z = 0.6;
    pose1.orientation.w = 1.0;

    obstacle1.primitives.push_back(primitive1);
    obstacle1.primitive_poses.push_back(pose1);
    obstacle1.operation = obstacle1.ADD;

    // Objeto 2: 
    moveit_msgs::msg::CollisionObject obstacle2;
    obstacle2.id = "obstacle2";
    obstacle2.header.frame_id = "panda_link0";

    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions = {0.1, 0.1, 0.1};

    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.1;
    pose2.position.y = 0.1;
    pose2.position.z = 1.1;
    pose2.orientation.w = 1.0;

    obstacle2.primitives.push_back(primitive2);
    obstacle2.primitive_poses.push_back(pose2);
    obstacle2.operation = obstacle2.ADD;

    // Añadir a la escena
    planning_scene_interface.applyCollisionObjects({obstacle1, obstacle2});

    while (rclcpp::ok()){
        std::cout << "\n--- MENÚ ---\n";
        std::cout << "1. Mover a Posición 1\n";
        std::cout << "2. Mover a Posición 2\n";
        std::cout << "3. Mover a Posición 3\n";
        std::cout << "4. Ingresar posición manual\n";
        std::cout << "5. Salir\n";
        std::cout << "Seleccione una opción: ";

        int opcion;
        std::cin >> opcion;

        geometry_msgs::msg::Pose target_pose;
        bool ejecutar = true;

        switch (opcion)
        {
            case 1:
                target_pose.position.x = -0.58;
                target_pose.position.y = 0.2;
                target_pose.position.z = 0.5;
                target_pose.orientation.w = 0.5;
                break;

            case 2:
                target_pose.position.x = 0.6;
                target_pose.position.y = 0.3;
                target_pose.position.z = 0.1;
                target_pose.orientation.w = 0.3;
                break;

            case 3:
                target_pose.position.x = 0.1;
                target_pose.position.y = 0.2;
                target_pose.position.z = 0.9;
                target_pose.orientation.w = 0.0;
                break;

            case 4:
                std::cout << "Ingrese x y z w separados por espacio: ";
                std::cin >> target_pose.position.x >> target_pose.position.y >> target_pose.position.z >> target_pose.orientation.w;
                break;

            case 5:
                std::cout << "Saliendo del programa.\n";
                rclcpp::shutdown();
                return 0;

            default:
                std::cout << "Opción inválida. Intente nuevamente.\n";
                ejecutar = false;
        }

        if (ejecutar){
            move_group.setPoseTarget(target_pose);
            auto result = move_group.move();

            if (!result)
            {
                std::cout << "Fallo al planear el movimiento. No se pudo alcanzar el objetivo.\n";
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}
