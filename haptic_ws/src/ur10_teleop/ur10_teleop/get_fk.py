import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK


class FKClientNode(Node):
    def __init__(self):
        super().__init__('fk_client_node')
        self.client = self.create_client(GetPositionFK, '/compute_fk')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/compute_fk service not available, waiting again...')
        self.req = GetPositionFK.Request()

    def send_request(self, header, fk_link_names, robot_state):
        self.req.header = header
        self.req.fk_link_names = fk_link_names
        self.req.robot_state = robot_state
        self.future = self.client.call_async(self.req)


class IKClientNode(Node):
    def __init__(self):
        super().__init__('ik_client_node')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/compute_ik service not available, waiting again...')
        self.req = GetPositionIK.Request()

    def send_request(self, ik_request):
        self.req.ik_request = ik_request
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    fk_client_node = FKClientNode()
    # Populate the request here based on your needs...

    header = Header()
    header.stamp = fk_client_node.get_clock().now().to_msg()  # Assuming you have a ROS node initialized
    header.frame_id = "base_link" 

    fk_link_names = ["base_link", "tool0"] 

    robot_state = RobotState()
    robot_state.joint_state = JointState()
    robot_state.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    robot_state.joint_state.position = [3.080001785198407, -2.3760839802651876,  -1.135113218857268, -1.214480224517489, 1.390300467660687, 1.3903042907211018] 


    fk_client_node.send_request(header, fk_link_names, robot_state)
    while rclpy.ok():
        rclpy.spin_once(fk_client_node)
        if fk_client_node.future.done():
            try:
                response = fk_client_node.future.result()
            except Exception as e:
                fk_client_node.get_logger().info('Service call failed %r' % (e,))
            else:
                # Process the response here
                print('Result of /compute_fk:', response)
            break
    rclpy.shutdown()

if __name__ == '__main__':
    main()