import rclpy
from .qvio_node import QVIO_Node

def main(args=None):
    rclpy.init(args=args)

    qvio_node = QVIO_Node()

    rclpy.spin(qvio_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qvio_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
