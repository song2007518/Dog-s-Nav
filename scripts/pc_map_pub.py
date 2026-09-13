import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


def load_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    m = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s+", data)
    if not m:
        raise ValueError("not a P5 pgm")
    width, height, maxval = int(m.group(1)), int(m.group(2)), int(m.group(3))
    header_len = m.end()
    pixels = data[header_len: header_len + width * height]
    if len(pixels) != width * height:
        raise ValueError("pgm size mismatch")
    return width, height, maxval, pixels


def parse_yaml(path):
    info = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            k, v = line.split(":", 1)
            k = k.strip()
            v = v.strip()
            if k == "origin":
                vals = [float(x) for x in v.strip("[]").split(",")[:3]]
                info[k] = vals
            elif k == "resolution":
                info[k] = float(v)
    return info


def main():
    rclpy.init()
    node = Node("pc_map_pub")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(OccupancyGrid, "/map", qos)

    pgm_path = "/tmp/opencode/map/map.pgm"
    yaml_path = "/tmp/opencode/map/map.yaml"

    # fall back to the persistent copy in the workspace if /tmp was cleared
    import os
    if not os.path.exists(pgm_path):
        pgm_path = os.path.expanduser("~/robot_ws/robot_sdk_ws/maps/map.pgm")
        yaml_path = os.path.expanduser("~/robot_ws/robot_sdk_ws/maps/map.yaml")

    width, height, maxval, pixels = load_pgm(pgm_path)
    yaml_info = parse_yaml(yaml_path)
    origin = yaml_info.get("origin", [-18.1, -16.95, 0.0])
    resolution = yaml_info.get("resolution", 0.05)

    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.position.z = origin[2]
    msg.data = [0] * (width * height)

    free_thresh = 0.2
    occupied_thresh = 0.5
    for row in range(height):
        for col in range(width):
            p = pixels[(height - 1 - row) * width + col]
            prob = (maxval - p) / maxval
            if prob > occupied_thresh:
                msg.data[row * width + col] = 100
            elif prob < free_thresh:
                msg.data[row * width + col] = 0
            else:
                msg.data[row * width + col] = -1

    node.get_logger().info(
        "publishing /map %dx%d res=%.2f origin=(%.1f,%.1f)" %
        (width, height, resolution, origin[0], origin[1]))

    rate = node.create_rate(1.0)
    while rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
