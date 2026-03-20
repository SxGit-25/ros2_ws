from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray

from ano_bridge.messages import (
    AttitudeMessage,
    FloatArrayMessage,
    QuaternionMessage,
    Vector3Message,
)


def build_vector3_stamped(stamp, frame_id: str, message: Vector3Message) -> Vector3Stamped:
    ros_message = Vector3Stamped()
    ros_message.header.stamp = stamp
    ros_message.header.frame_id = frame_id
    ros_message.vector.x = message.x
    ros_message.vector.y = message.y
    ros_message.vector.z = message.z
    return ros_message


def build_attitude_stamped(stamp, frame_id: str, message: AttitudeMessage) -> Vector3Stamped:
    ros_message = Vector3Stamped()
    ros_message.header.stamp = stamp
    ros_message.header.frame_id = frame_id
    ros_message.vector.x = message.roll_deg
    ros_message.vector.y = message.pitch_deg
    ros_message.vector.z = message.yaw_deg
    return ros_message


def build_quaternion_stamped(
    stamp,
    frame_id: str,
    message: QuaternionMessage,
) -> QuaternionStamped:
    ros_message = QuaternionStamped()
    ros_message.header.stamp = stamp
    ros_message.header.frame_id = frame_id
    ros_message.quaternion.w = message.w
    ros_message.quaternion.x = message.x
    ros_message.quaternion.y = message.y
    ros_message.quaternion.z = message.z
    return ros_message


def build_float_array(message: FloatArrayMessage) -> Float32MultiArray:
    ros_message = Float32MultiArray()
    ros_message.data = message.data
    return ros_message
