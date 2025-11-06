#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String # Mensagens do topico gancho serão do tipo string


class GanchoReleaseNode(Node):
    """
    Responsável para desatirvar o eletroíma e soltar o gancho ao receber a mensagem de drone
    centralizado em *'/gancho/posicao_drone'*
    """

    def __init__(self):
        super().__init__(node_name='gancho_release_node')
        
        self.ready = False      # Se drone está pronto para ter gancho solto
        self.status = 'waiting' # Status do módulo gancho


        # Publicará status do gancho ex: 'preparing', 'released'
        self.status_publisher = self.create_publisher(msg_type=String,
                                                      topic='/gancho/status',
                                                      qos_profile=10)
        
        # Receberá aviso que drone está devidamente centralizado acima do fio
        self.drone_state_subscriber = self.create_subscription(msg_type=String,
                                                               callback=self.drone_state_cb,
                                                               topic='/gancho/posicao_drone',
                                                               qos_profile=10)
        
        self.get_logger().info('GanchoReleaseNode inicializado')


    # ------------- #
    #   CALLBACKS   #
    # ------------- #
    def drone_state_cb(self, msg):
        if msg.data == 'Drone centralizado':
            self.ready = True    
    
        self.get_logger().info(f'received {msg.data}')
    


#===================#
#                   #
#       Main        #
#                   #
#===================#
def main(args=None):
    rclpy.init(args=args)
    node = GanchoReleaseNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
