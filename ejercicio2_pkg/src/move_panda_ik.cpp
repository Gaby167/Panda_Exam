#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <iostream>
using namespace std;

void addObstacles(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::msg::CollisionObject box1, box2;
    box1.id = "box1";
    box1.header.frame_id = "panda_link0";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.2, 0.2};  // Tamaño del cubo

    geometry_msgs::msg::Pose box1_pose;
    box1_pose.position.x = 0.4;
    box1_pose.position.y = 0.0;
    box1_pose.position.z = 0.3;
    box1_pose.orientation.w = 1.0;

    box1.primitives.push_back(primitive);
    box1.primitive_poses.push_back(box1_pose);
    box1.operation = box1.ADD;

    box2 = box1;
    box2.id = "box2";
    box2.primitive_poses[0].position.x = 0.0;
    box2.primitive_poses[0].position.y = -0.4;

    planning_scene_interface.applyCollisionObjects({box1, box2});
    RCLCPP_INFO(rclcpp::get_logger("move_panda"), "Obstáculos añadidos");
}

geometry_msgs::msg::Pose createPose(double x, double y, double z, double w)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Usamos orientación simplificada
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = w;

    return pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_panda_ik_node");

    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    addObstacles(planning_scene_interface);

    while (rclcpp::ok()) {
        cout << "\n--- MENÚ ---" << endl;
        cout << "1. Mover a Posición 1" << endl;
        cout << "2. Mover a Posición 2" << endl;
        cout << "3. Mover a Posición 3" << endl;
        cout << "4. Ingresar posición manual" << endl;
        cout << "5. Salir" << endl;
        cout << "Seleccione una opción: ";

        int opcion;
        cin >> opcion;

        geometry_msgs::msg::Pose target_pose;

        if (opcion == 1) {
            target_pose = createPose(-0.58, 0.2, 0.5, 0.5);
        } else if (opcion == 2) {
            target_pose = createPose(0.9, 0.4, 0.1, 0.3);
        } else if (opcion == 3) {
            target_pose = createPose(0.1, 0.2, 0.9, 0.0);
        } else if (opcion == 4) {
            double x, y, z, w;
            cout << "Ingrese x y z w separados por espacio: ";
            cin >> x >> y >> z >> w;
            target_pose = createPose(x, y, z, w);
        } else if (opcion == 5) {
            break;
        } else {
            cout << "Opción inválida." << endl;
            continue;
        }

        move_group.setPoseTarget(target_pose);

        auto success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
            cout << "Movimiento exitoso." << endl;
        else
            cout << "Fallo al planear el movimiento. No se pudo alcanzar el objetivo." << endl;
    }

    rclcpp::shutdown();
    return 0;
}
