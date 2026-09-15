import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from threading import Thread


def getch():
    """Read a single character from stdin without echo."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.start_client = self.create_client(Trigger, 'start_collect')
        self.stop_client = self.create_client(Trigger, 'stop_collect')
        self.get_logger().info("Keyboard controller ready! Keys: [s] start  [d] stop  [q] quit")

    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'{name} service unavailable')
            return
        future = client.call_async(Trigger.Request())

        def callback(future):
            try:
                resp = future.result()
                if resp.success:
                    self.get_logger().info(f"{name}: {resp.message}")
                else:
                    self.get_logger().warn(f"{name} failed: {resp.message}")
            except Exception as e:
                self.get_logger().error(f"{name} error: {e}")

        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()

    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('Keys: [s] start collecting  [d] stop collecting  [q] quit')

    try:
        while True:
            ch = getch()
            if ch == 's':
                node.call_service(node.start_client, 'start_collect')
            elif ch == 'd':
                node.call_service(node.stop_client, 'stop_collect')
            elif ch in ('q', '\x03'):  # q or Ctrl-C
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
