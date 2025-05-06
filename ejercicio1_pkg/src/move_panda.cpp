#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_panda_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Interfaz para el brazo Panda
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

    // Nombre del planner
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Velocidad y aceleración reducida
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    while (rclcpp::ok()) {
        cout << "\n--- MENÚ ---" << endl;
        cout << "1. Mover a Posición 1" << endl;
        cout << "2. Mover a Posición 2" << endl;
        cout << "3. Ingresar posición manualmente" << endl;
        cout << "4. Salir" << endl;
        cout << "Seleccione una opción: ";

        int opcion;
        cin >> opcion;

        std::vector<double> joint_values(7);
        
        switch (opcion) {
            case 1:
                joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 3.14, 0.78};
                break;

            case 2:
                joint_values = {0.0, 1.57, 0.0, 0.0, 0.0, 3.14, 0.78};
                break;

            case 3:
                cout << "Ingrese 7 valores de articulaciones separados por espacio:" << endl;
                for (int i = 0; i < 7; ++i) {
                    cin >> joint_values[i];
                }
                break;

            case 4:
                rclcpp::shutdown();
                return 0;

            default:
                cout << "Opción inválida." << endl;
                continue;
        }

        move_group.setJointValueTarget(joint_values);

        // Planear antes de mover
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group.plan(plan));

        if (success) {
            cout << "Plan exitoso, ejecutando..." << endl;
            move_group.execute(plan);
            cout << "Movimiento completado." << endl;
        } else {
            cout << "Falló el planeamiento del movimiento." << endl;
        }

        executor.spin_some();  // Procesa callbacks internos
    }

    rclcpp::shutdown();
    return 0;
}
