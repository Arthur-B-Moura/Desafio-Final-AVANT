#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String # Mensagens do topico gancho serão do tipo string
import threading


class TesterNode(Node):
    """ 
    ### Node para teste da funcionalidade do gancho:
    
    Simula envio de mensagem do drone centralizado para permitir testes modularizados
    das funções do módulo do gancho, sem depender da completude dos módulos da Etapa 1 do desafio
    
    > Esse node contará com um publisher que publicará a mensagem devida no tópico devido após
    algum input qualquer no terminal
    """
    
    def __init__(self):
        super().__init__(node_name='tester_node')

        self.msg_publisher = self.create_publisher(msg_type=String,
                                                   topic='/gancho/posicao_drone',
                                                   qos_profile=10)
        self.get_logger().info('Publisher node created. Enter messages to publish, or type "exit" to quit.')


    def publish_test_message(self):
        msg = String()
        msg.data = "Drone centralizado" # Mensagem padrão para quando o drone estiver
                                        # posicionado centralizado acima do fio vermelho
        
        self.msg_publisher.publish(msg)
        self.get_logger().info(f'Sent "{msg.data}" to "/gancho/posicao_drone"')


#===================#
#                   #
#       Main        #
#                   #
#===================#
def main(args=None):
    rclpy.init(args=args)
    node = TesterNode()

    # Create a separate thread for spinning the ROS 2 node
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()

    try:
        while rclpy.ok():
            user_input = input("")
            if user_input.lower() == 'exit':
                break
            node.publish_test_message()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()


if __name__ ==  '__main__':
    main()