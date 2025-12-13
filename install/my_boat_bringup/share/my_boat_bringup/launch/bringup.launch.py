import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        
        # 1. Boat Motor Driver
        # Runs in the background, prints logs to this terminal
        launch_ros.actions.Node(
            package='my_boat_driver',
            executable='motor_driver_node',
            name='boat_base_controller',
            output='screen'
        ),

        # 2. Teleop Keyboard
        # Opens in a NEW xterm window so you can type keys
        launch_ros.actions.Node(
            package='my_boat_teleop',
            executable='teleop_keyboard_node',
            name='teleop_keyboard_node',
            output='screen',
            # 'prefix' opens this specific node in a new terminal window
            prefix='xterm -e'
        )
    ])