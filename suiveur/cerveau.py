import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .metier import CalculateurCerveauDistance, CalculateurCerveauOrientation

from math import radians as degrees_to_radians

class CerveauNode(Node):
    """Noeud ROS qui commande le robot à partir des mesures du laser."""

    def __init__(self):
        super().__init__('cerveau')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.calculateur_orientation = CalculateurCerveauOrientation()
        self.calculateur_orientation.angle_vue = degrees_to_radians(60.0)

        self.calculateur_distance = CalculateurCerveauDistance(distance_cible=0.3)

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data,
        )

    def timer_callback(self):
        msg = Twist()
        a = self.calculateur_orientation.calculer_correction()
        msg.angular.z = a
        self.get_logger().info('Correction angle : %.3f m' % a)

        condition_orientation_valid = abs(self.calculateur_orientation.erreur) < 0.8

        if condition_orientation_valid:
            c = self.calculateur_distance.calculer_correction()
            msg.linear.x = c
            msg.angular.z = 0.0
            self.get_logger().info('Correction distance : %.3f m' % c)
        
        self.publisher_.publish(msg)


    def laser_callback(self, msg):
        angle = self.calculateur_orientation.mettre_a_jour_erreur(
            msg.ranges, msg.angle_min, msg.angle_increment
        )
        if angle is None:
            self.get_logger().warning('Laser scan contains no finite ranges')
            return

        distance = self.calculateur_distance.mettre_a_jour_erreur(
            msg.ranges, msg.angle_min, msg.angle_increment
        )

        if distance is None:
            self.get_logger().warning('Laser scan contains no finite ranges')
            return
        
        # self.get_logger().info('Minimum distance is: %.3f m' % distance)


def main(args=None):
    """Initialise et exécute le noeud cerveau."""
    rclpy.init(args=args)
    cerveau_node = CerveauNode()
    try:
        rclpy.spin(cerveau_node)
    finally:
        cerveau_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
