"""Republish a captured description for a reversible model-event smoke check."""
import argparse
import json
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--capture',required=True)
    parser.add_argument('--topic',default='/robot_description')
    parser.add_argument('--restore',action='store_true')
    args=parser.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from std_msgs.msg import String
    original=json.load(open(args.capture))['description']
    data=original if args.restore else original+'\n<!-- reversible conformance model event -->\n'
    rclpy.init();node=rclpy.create_node('conformance_description_restore' if args.restore else 'conformance_description_change')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    publisher=node.create_publisher(String,args.topic,qos)
    try:
        publisher.publish(String(data=data))
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
