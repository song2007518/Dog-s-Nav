import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

SERVICE = sys.argv[1] if len(sys.argv) > 1 else "/zs1w/lie_down"


def main():
    rclpy.init()
    node = Node("sdk_srv_call")
    cli = node.create_client(Trigger, SERVICE)
    if not cli.wait_for_service(timeout_sec=8):
        node.get_logger().error("%s not available" % SERVICE)
        rclpy.shutdown()
        return 1
    req = Trigger.Request()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    if fut.done():
        r = fut.result()
        node.get_logger().info("%s -> success=%s msg=%s" % (SERVICE, r.success, r.message))
    else:
        node.get_logger().error("%s call timeout" % SERVICE)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
