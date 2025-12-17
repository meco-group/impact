#!/usr/bin/env python3
"""
Synchronous closed-loop driver for the cart_pendros nodes (no realtime).

Loop per step k:
  1) publish x_k   -> /impact/x_current
  2) wait for u0   <- /impact/control_to_apply    (data[0])
  3) wait for x_{k+1} <- /model/x_next
  4) log [k, t_rel, u0, x_k]; set x_k = x_{k+1}
"""

import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray

# Topic names as in your nodes
TOPIC_XCURR = '/impact/x_current'
TOPIC_U     = '/impact/control_to_apply'
TOPIC_XNEXT = '/impact/x_next'


class ClosedLoop(Node):
    """Minimal synchronous supervisor with mailbox-style callbacks."""

    def __init__(self, x0, steps, csv_path):
        super().__init__('mpc_closed_loop_sync')

        self.xk = list(x0)       # current state (len 4)
        self.steps = int(steps)
        self.csv_path = csv_path

        # Reliable QoS; KEEP_LAST is enough for point-to-point
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # I/O
        self.pub_xcurr = self.create_publisher(Float32MultiArray, TOPIC_XCURR, qos)
        self.sub_u     = self.create_subscription(Float32MultiArray, TOPIC_U,     self._on_u, qos)
        self.sub_xnext = self.create_subscription(Float32MultiArray, TOPIC_XNEXT, self._on_x, qos)

        # Mailboxes (None => not received yet)
        self.u0 = None
        self.x_next = None

        # Logging
        self.log = []            # rows: [k, t_rel, u0, x0, x1, x2, x3]
        self.t0 = time.time()

        # Small warm-up so subscriptions connect before first publish
        self.get_logger().info('Waiting 0.5 s for connections…')
        t_end = time.time() + 0.5
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)


    # ---- Callbacks put data into mailboxes ----
    def _on_u(self, msg: Float32MultiArray):
        if msg.data:
            self.u0 = float(msg.data[0])  # controller gives scalar in data[0]

    def _on_x(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.x_next = [float(msg.data[0]), float(msg.data[1]),
                           float(msg.data[2]), float(msg.data[3])]

    # ---- One synchronous iteration ----
    def _do_one_step(self, k: int):
        # 1) publish x_k
        m = Float32MultiArray(); m.data = self.xk
        self.pub_xcurr.publish(m)

        # 2) wait for u0 from controller
        self.u0 = None
        while rclpy.ok() and self.u0 is None:
            rclpy.spin_once(self, timeout_sec=0.05)

        # 3) wait for x_{k+1} from model
        self.x_next = None
        while rclpy.ok() and self.x_next is None:
            rclpy.spin_once(self, timeout_sec=0.05)

        # 4) log and feed back
        t_rel = time.time() - self.t0
        self.log.append([k, t_rel, self.u0] + list(self.xk))
        self.xk = self.x_next
        self.get_logger().info(f'k={k:03d}  u0={self.u0:.6f}  x={self.xk}')

    # ---- Run N iterations and write CSV ----
    def run(self):
        try:
            for k in range(self.steps):
                self._do_one_step(k)
        finally:
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['k', 't_rel', 'u0', 'x0', 'x1', 'x2', 'x3'])
                w.writerows(self.log)
            self.get_logger().info(f'Wrote {self.csv_path} with {len(self.log)} rows.')


def main():
    ap = argparse.ArgumentParser(description='Synchronous closed loop for cart_pendros.')
    ap.add_argument('--x0', nargs=4, type=float, required=True,
                    help='Initial state [pos theta dpos dtheta]')
    ap.add_argument('--steps', type=int, default=70, help='Number of iterations')
    ap.add_argument('--csv', type=str, default='mpc_log.csv', help='Output CSV path')
    args = ap.parse_args()

    rclpy.init()
    node = ClosedLoop(args.x0, args.steps, args.csv)
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()









# import argparse, csv, time
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from std_msgs.msg import Float32MultiArray

# TOPIC_XCURR  = '/impact/x_current'
# TOPIC_U      = '/impact/control_to_apply'
# TOPIC_XNEXT  = '/model/x_next'

# class ClosedLoop(Node):
#     def __init__(self, x0, steps, csv_path):
#         super().__init__('mpc_closed_loop_sync')
#         self.steps   = steps
#         self.csv_path = csv_path

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # pubs / subs
#         self.pub_xcurr = self.create_publisher(Float32MultiArray, TOPIC_XCURR, qos)
#         self.sub_u     = self.create_subscription(Float32MultiArray, TOPIC_U, self.cb_u, qos)
#         self.sub_xnext = self.create_subscription(Float32MultiArray, TOPIC_XNEXT, self.cb_x, qos)

#         # state
#         self.xk = list(x0)          # current state
#         self.u0 = None              # latest control to apply (scalar in data[0])
#         self.x_next = None          # next state received from model
#         self.waiting_for_u = False
#         self.waiting_for_x = False

#         # logging
#         self.log = []               # rows: [k, t_rel, u0, x0, x1, x2, x3]
#         self.t0 = time.time()
#         self.k  = 0

#         # ensure subscribers are connected before first publish (best-effort)
#         self.get_logger().info('Warming up for 0.5s so subscribers can connect…')
#         t_end = time.time() + 0.5
#         while rclpy.ok() and time.time() < t_end:
#             rclpy.spin_once(self, timeout_sec=0.05)

#         self.get_logger().info('Starting closed loop…')
#         self.run_loop()

#     # ----- Callbacks -----
#     def cb_u(self, msg: Float32MultiArray):
#         if not self.waiting_for_u:
#             return
#         if not msg.data:
#             return
#         self.u0 = float(msg.data[0])  # controller publishes a scalar in data[0]
#         self.waiting_for_u = False

#     def cb_x(self, msg: Float32MultiArray):
#         if not self.waiting_for_x:
#             return
#         if len(msg.data) < 4:
#             return
#         self.x_next = list(msg.data[:4])
#         self.waiting_for_x = False

#     # ----- One synchronous step -----
#     def do_step(self):
#         # 1) publish x_current
#         xmsg = Float32MultiArray()
#         xmsg.data = self.xk
#         self.pub_xcurr.publish(xmsg)

#         # 2) wait for controller's control_to_apply (u0)
#         self.u0 = None
#         self.waiting_for_u = True
#         while rclpy.ok() and self.waiting_for_u:
#             rclpy.spin_once(self, timeout_sec=0.05)

#         # 3) wait for model's x_next
#         self.x_next = None
#         self.waiting_for_x = True
#         while rclpy.ok() and self.waiting_for_x:
#             rclpy.spin_once(self, timeout_sec=0.05)

#         # 4) log, update xk
#         t_rel = time.time() - self.t0
#         row = [self.k, t_rel, self.u0] + list(self.xk)
#         self.log.append(row)
#         # feed back
#         self.xk = self.x_next

#     def run_loop(self):
#         try:
#             for self.k in range(self.steps):
#                 self.do_step()
#                 self.get_logger().info(
#                     f"k={self.k:03d}  u0={self.u0:.6f}  x={self.xk}"
#                 )
#         finally:
#             # write csv
#             with open(self.csv_path, 'w', newline='') as f:
#                 w = csv.writer(f)
#                 w.writerow(['k','t_rel','u0','x0','x1','x2','x3'])
#                 for r in self.log:
#                     w.writerow(r)
#             self.get_logger().info(f"Wrote {self.csv_path} with {len(self.log)} rows.")
#             rclpy.shutdown()

# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--x0', nargs=4, type=float, required=True,
#                         help='Initial state [pos theta dpos dtheta]')
#     parser.add_argument('--steps', type=int, default=70)
#     parser.add_argument('--csv', type=str, default='mpc_log.csv')
#     args = parser.parse_args()

#     rclpy.init()
#     ClosedLoop(args.x0, args.steps, args.csv)
#     # Node shuts down itself after finishing

# if __name__ == '__main__':
#     main()