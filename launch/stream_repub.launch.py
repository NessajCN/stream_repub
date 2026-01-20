import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
# from tracetools_launch.action import Trace


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="stream_repub",
                executable="repub",
                namespace="stereo_left",
                name="image_repub",
                # remappings=[("images_raw", "left/image_raw")],
                parameters=[
                    {
                        "stream_url": "http://192.168.11.3:1985/rtc/v1/whep/?app=sensor/camera/stereo_left&stream=image"
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="stream_repub",
                executable="repub",
                namespace="stereo_right",
                name="image_repub",
                # remappings=[("image_raw", "right/image_raw")],
                parameters=[
                    {
                        "stream_url": "http://192.168.11.3:1985/rtc/v1/whep/?app=sensor/camera/stereo_right&stream=image"
                    }
                ],
            ),
        ]
    )
