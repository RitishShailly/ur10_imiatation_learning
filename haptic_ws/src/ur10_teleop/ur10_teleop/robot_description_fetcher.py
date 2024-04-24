import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from urdf_parser_py.urdf import URDF
# import kdl_parser_py.urdf

class FetchURDF(Node):
    def __init__(self):
        super().__init__('fetch_urdf')
        self.cli = self.create_client(GetParameters, '/move_group/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/move_group/get_parameters service not available, waiting again...')
        self.req = GetParameters.Request()

    def get_robot_description(self):
        self.req.names = ['robot_description']
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        try:
            response = self.future.result()
            robot_description_string = response.values[0].string_value
            return URDF.from_xml_string(robot_description_string)
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
            return None

def write_robot_description_to_file(description, file_path):
    try:
        with open(file_path, 'w') as file:
            file.write(description.to_xml_string())
        print(f"URDF saved to {file_path}")
    except Exception as e:
        print(f"Failed to save URDF to file: {e}")

def main(args=None):
    rclpy.init(args=args)
    fetch_urdf_node = FetchURDF()
    robot_urdf_model = fetch_urdf_node.get_robot_description()
    if robot_urdf_model is not None:
        # Save the URDF to a file
        write_robot_description_to_file(robot_urdf_model, "/home/rshailly/Documents/ur10_description.urdf")
    # robot_kdl = get_kdl_tree(robot_urdf_model)
    # print(robot_kdl)

    fetch_urdf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

