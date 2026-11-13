"""Explicit simulator actuator-stall fixture; publish a held intermediate target."""
import argparse
import time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--topic',required=True)
    parser.add_argument('--position',required=True,type=float)
    parser.add_argument('--stop-file',required=True)
    parser.add_argument('--duration',type=float,default=5.)
    args=parser.parse_args()
    stop_file=Path(args.stop_file)
    stop_file.unlink(missing_ok=True)
    import rclpy
    from std_msgs.msg import Float64MultiArray
    rclpy.init();node=rclpy.create_node('conformance_simulated_tool_stall')
    publisher=node.create_publisher(Float64MultiArray,args.topic,10)
    try:
        deadline=time.monotonic()+args.duration
        print("STALL FIXTURE READY",flush=True)
        while rclpy.ok() and time.monotonic()<deadline and not stop_file.exists():
            publisher.publish(Float64MultiArray(data=[args.position]))
            rclpy.spin_once(node,timeout_sec=.025)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
