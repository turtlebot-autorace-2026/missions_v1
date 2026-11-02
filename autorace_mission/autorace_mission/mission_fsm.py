'''
mission_fsm node: the only place that decides what the robot is doing.

  sub  <left_topic>, <right_topic>   std_msgs/Bool   sign detector (True = sign confirmed)
  sub  <odom_topic>                  nav_msgs/Odometry
  pub  /lane_state                   std_msgs/Bool   lane node on/off
  pub  /lane/follow_line             std_msgs/String both | yellow | white
  pub  /intersection_state           std_msgs/Bool   lane node slow-down flag
  pub  /mission/drive                std_msgs/String lane | mission | stop -> cmd_vel_mux
  (all re-sent at rate_hz; the lane node needs lane_detection_ad with edge-triggered /lane_state)
  pub  /mission/state                std_msgs/String human-readable status
'''
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from autorace_mission.fsm_core import DEFAULTS, MissionFSM


class MissionFsmNode(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        p = self.declare_parameter
        p('left_topic', '/left')
        p('right_topic', '/right')
        p('odom_topic', '/odom')
        p('rate_hz', 10.0)
        p('status_period', 1.0)
        for k, v in DEFAULTS.items():
            p(k, v)
        self.fsm = MissionFSM(**{k: self.get_parameter(k).value for k in DEFAULTS})

        left, right = self.get_parameter('left_topic').value, self.get_parameter('right_topic').value
        self.create_subscription(Bool, left, lambda m: self.fsm.on_sign('left', m.data, self.now()), 10)
        self.create_subscription(Bool, right, lambda m: self.fsm.on_sign('right', m.data, self.now()), 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)

        self.pub_lane_state = self.create_publisher(Bool, '/lane_state', 10)
        self.pub_follow = self.create_publisher(String, '/lane/follow_line', 10)
        self.pub_slow = self.create_publisher(Bool, '/intersection_state', 10)
        self.pub_drive = self.create_publisher(String, '/mission/drive', 10)
        self.pub_state = self.create_publisher(String, '/mission/state', 10)

        self.lane_state_sent = None
        self.last_status_t = -1e9
        self.status_period = self.get_parameter('status_period').value
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.get_logger().info('mission_fsm: signs %s / %s, params %s' % (left, right, self.fsm.p))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.fsm.on_odom(pos.x, pos.y)

    def tick(self):
        t = self.now()
        out = self.fsm.step(t)
        for when, text in self.fsm.events:
            self.get_logger().info(text)
        self.fsm.events = []

        # everything is re-sent every tick (a single lost message must not leave the robot standing);
        # the lane node acts on /lane_state edges only, so repeats do not reset its estimator
        if out.lane_on != self.lane_state_sent:
            self.get_logger().info('/lane_state %s' % out.lane_on)
            self.lane_state_sent = out.lane_on
        self.pub_lane_state.publish(Bool(data=out.lane_on))
        self.pub_follow.publish(String(data=out.follow_line))
        self.pub_slow.publish(Bool(data=out.slow))
        self.pub_drive.publish(String(data=out.drive))

        status = self.fsm.status(t) + ' | out %s' % out.as_dict()
        self.pub_state.publish(String(data=status))
        if t - self.last_status_t >= self.status_period:
            self.last_status_t = t
            self.get_logger().info(status)


def main():
    rclpy.init()
    node = MissionFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_drive.publish(String(data='stop'))
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
