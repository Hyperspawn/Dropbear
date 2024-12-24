
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Leg controllers
    right_leg_cont = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_leg_controller",
                "--controller-manager", "/controller_manager",
            ]

    )
       
    left_leg_cont = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "left_leg_controller",
                "--controller-manager", "/controller_manager",
            ]

    )

    # Hand Controllers
    right_hand_cont = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_hand_controller",
                "--controller-manager", "/controller_manager",
            ]
    )

    left_hand_cont = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "left_hand_controller",
                "--controller-manager", "/controller_manager",
            ]

    )

    stewart_prismatic_cont = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                "stewart_slider_controller",
                "--controller-manager", "/controller_manager",
            ] 
    )

    waist_cont = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                "waist_joint_controller",
                "--controller-manager", "/controller_manager",
            ] 
    )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ]
        )


    return LaunchDescription([
        joint_state_broadcaster,
        # stewart_prismatic_cont,
        right_hand_cont,
        left_hand_cont,
        waist_cont,
        right_leg_cont,
        left_leg_cont,
    ])