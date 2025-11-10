#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String # Mensagens do topico gancho serão do tipo string

import Jetson.GPIO as GPIO 


"""
Subscriber
- Tópico: '/gancho/posicao_drone'
    -> mensagem String() 
    -> Mensagem avisando que drone esta centralizado
    -> msg.data esperado = 'Drone centralizado'

Publisher
- Topico: '/gancho/status'
    -> mensagem String()
    -> mensagem com updates do status do gancho
    -> msg.data esperados:
        >> 'waiting'   --> aguardando drone centralizado
        >> 'preparing' --> recebida mensagem de drone centralizado, mas ganchho ainda não foi solto
        >> 'released'  --> gancho foi solto, drone pode deixar a posição e terminar a missão
"""


# --------------------- #
#   SETUP PINO JETSON   #
# --------------------- #
    # Relé considerado (Modo NF, então pino HIGH = chave aberta):
    # https://www.eletrogate.com/modulo-rele-1-canal-5v?utm_source=Site&utm_medium=GoogleMerchant&utm_campaign=GoogleMerchant&gad_source=1&gad_campaignid=22374767015&gclid=CjwKCAiAzrbIBhA3EiwAUBaUdV65XmIFcpJ1tEooZ0ZfQOorHIS9c4J-pEPjll5WSVHxk4zszqpS_xoCJ-oQAvD_BwE
PINO_CONTROLE_RELE = 7 # Pino GPIO que controla comportamento do relé

GPIO.setmode(GPIO.BOARD) 
GPIO.setup(PINO_CONTROLE_RELE,GPIO.OUT) # Define pino do relé como output


# -------------------- #
#   SETUP NODE ROS2    #
# -------------------- #
class GanchoReleaseNode(Node):
    """
    Responsável para desatirvar o eletroíma e soltar o gancho ao receber a mensagem de drone
    centralizado em *'/gancho/posicao_drone'*
    """

    def __init__(self):
        super().__init__(node_name='gancho_release_node')
        
        self.status = String()
        self.status.data = 'waiting' # Status do módulo gancho


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



    # ----------- #
    #   PUBLISH   #
    # ----------- #
    def publica_status(self):
        """ Publica status atual do lançamento do gancho no tópico devido """

        self.status_publisher.publish(self.status)
        self.get_logger().info(f'Publicado {self.status.data}')


    # ------------- #
    #   CALLBACKS   #
    # ------------- #
    def drone_state_cb(self, msg):
        if msg.data == 'Drone centralizado' and self.status.data == 'waiting':
            self.get_logger().info(f'received {msg.data}')
            self.status.data = 'preparing'

            self.libera_rele_gancho()
    
    

    # ----------------------- #
    #   FUNÇÕES DE CONTROLE   #
    # ----------------------- #
    def libera_rele_gancho(self):
        """ Função que controla interação física entre os pinos GPIO da
         Jetson Orin Nano para controlar o relé e soltar o gancho. """

        # Sinal do pino HIGH, o que (de acordo com a configuração considerada para
        # o relé), o deixará na configuração de chave aberta, soltando o gancho
        GPIO.output(PINO_CONTROLE_RELE, GPIO.HIGH)

        # Atualiza status, avisando que gancho já foi solto
        self.status.data = 'released' 
        self.get_logger().info("Relé desativado -> gancho liberado")

        # Desabilita self.ready para evitar que função seja chamada mais de uma vez
        self.ready = False
    


#===================#
#                   #
#       Main        #
#                   #
#===================#
def main(args=None):
    rclpy.init(args=args)
    node = GanchoReleaseNode()

    # Publica status do gancho a cada 1s     
    node.create_timer(2.0, node.publica_status)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
