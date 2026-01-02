import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

def main():
    rclpy.init()
    node = Node('spawn_turtlebot')
    cli = node.create_client(SpawnEntity, '/world/world_demo/spawn_entity')
    
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for spawn service...')

    req = SpawnEntity.Request()
    req.name = 'turtlebot'
    req.xml = open('/path/to/turtlebot/model.sdf', 'r').read()  # path to your SDF
    req.robot_namespace = 'turtlebot'

    # Set initial pose
    req.initial_pose = Pose()
    req.initial_pose.position.x = 0
    req.initial_pose.position.y = 0
    req.initial_pose.position.z = 0
    req.initial_pose.orientation.x = 0
    req.initial_pose.orientation.y = 0
    req.initial_pose.orientation.z = 0
    req.initial_pose.orientation.w = 1

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info('TurtleBot spawned.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

