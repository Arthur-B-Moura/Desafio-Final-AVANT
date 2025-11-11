#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import math

# Mensagens ROS
from std_msgs.msg import String, Int32MultiArray, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, Twist


class NavigatorNode(Node):
    """
    Nó principal de navegação que integra a visão e o controle do gancho.
    Controla o drone com base em uma máquina de estados.
    """

    def __init__(self):
        super().__init__('navigator_node')

        # --- Parâmetros ---
        self.declare_parameter('altitude_cruzeiro', 3.0)
        self.declare_parameter('ganho_p_linear', 0.005)  # Ganho Proporcional para velocidade linear
        self.declare_parameter('ganho_p_angular', 0.002)  # Ganho Proporcional para velocidade angular
        self.declare_parameter('tolerancia_centralizacao', 25)  # Pixels de tolerância para o alvo

        self.ALTITUDE_CRUZEIRO = self.get_parameter('altitude_cruzeiro').get_parameter_value().double_value
        self.KP_LINEAR = self.get_parameter('ganho_p_linear').get_parameter_value().double_value
        self.KP_ANGULAR = self.get_parameter('ganho_p_angular').get_parameter_value().double_value
        self.CENTERING_TOLERANCE = self.get_parameter('tolerancia_centralizacao').get_parameter_value().integer_value

        # --- Variáveis de Estado ---
        self.mission_state = "INICIO"  # Máquina de estados: INICIO, DECOLANDO, SEGUINDO_LINHA, CENTRALIZANDO, SOLTANDO_GANCHO, VOLTANDO, POUSANDO, FIM
        self.current_fcu_state = State()
        self.current_pose = None
        self.vision_state = "perdido"
        self.line_center = [-1, -1]
        self.red_detected = False
        self.gancho_status = "waiting"
        self.home_position = None  # Posição de decolagem para retorno

        # --- Configuração de ROS ---
        self.callback_group = ReentrantCallbackGroup()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.gancho_pos_pub = self.create_publisher(String, '/gancho/posicao_drone', 10)

        # --- Subscribers ---
        # Callbacks de MAVROS
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)

        # Callbacks de Visão
        self.vision_state_sub = self.create_subscription(String, '/vision/state', self.vision_state_cb, 10)
        self.line_pos_sub = self.create_subscription(Int32MultiArray, '/vision/line_position', self.line_position_cb,
                                                     10)

        # Callback do Gancho
        self.gancho_status_sub = self.create_subscription(String, '/gancho/status', self.gancho_status_cb, 10)

        # --- Clients de Serviço MAVROS ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # --- Timers ---
        self.mission_timer = self.create_timer(0.1, self.mission_step)  # Loop principal da missão

        self.get_logger().info(f"Nó de navegação iniciado. Altitude de cruzeiro: {self.ALTITUDE_CRUZEIRO}m")

    # --- Funções de Callback ---
    def state_cb(self, msg):
        self.current_fcu_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        if self.home_position is None:  # Salva a primeira posição recebida como 'home'
            self.home_position = self.current_pose.position
            self.get_logger().info(
                f"Posição HOME definida em: x={self.home_position.x:.2f}, y={self.home_position.y:.2f}")

    def vision_state_cb(self, msg):
        self.vision_state = msg.data

    def line_position_cb(self, msg):
        self.line_center = msg.data

    def gancho_status_cb(self, msg):
        self.gancho_status = msg.data

    # --- Funções de Controle MAVROS (Assíncronas) ---
    async def set_mode(self, mode_name):
        await self.set_mode_client.wait_for_service()
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.set_mode_client.call_async(req)
        await future
        if future.result().mode_sent:
            self.get_logger().info(f"Modo alterado para {mode_name}")
            return True
        self.get_logger().error(f"Falha ao alterar modo para {mode_name}")
        return False

    async def arm(self):
        await self.arming_client.wait_for_service()
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        await future
        if future.result().success:
            self.get_logger().info("Drone armado")
            return True
        self.get_logger().error("Falha ao armar o drone")
        return False

    async def takeoff(self, altitude):
        await self.takeoff_client.wait_for_service()
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        await future
        if future.result().success:
            self.get_logger().info(f"Decolando para {altitude}m")
            return True
        self.get_logger().error("Falha na decolagem")
        return False

    async def land(self):
        await self.land_client.wait_for_service()
        req = CommandTOL.Request()
        future = self.land_client.call_async(req)
        await future
        if future.result().success:
            self.get_logger().info("Iniciando pouso")
            return True
        self.get_logger().error("Falha ao iniciar pouso")
        return False

    # --- Lógica de Navegação ---
    def follow_line(self):
        """Controlador Proporcional para seguir a linha."""
        if self.line_center[0] == -1 or self.vision_state == 'perdido':
            # Se a linha for perdida, para de se mover para frente e gira para procurar
            self.send_velocity(linear_x=0.0, angular_z=0.3)
            self.get_logger().warn("Linha perdida, procurando...", throttle_duration_sec=1)
            return

        # O centro da imagem da câmera é assumido como 320 para uma imagem de 640 de largura.
        # Ajuste se sua câmera tiver resolução diferente.
        image_center_x = 320
        error = self.line_center[0] - image_center_x

        linear_x = 0.5  # Velocidade constante para frente
        angular_z = -self.KP_ANGULAR * error

        self.send_velocity(linear_x=linear_x, angular_z=angular_z)

    def center_on_target(self):
        """Controlador Proporcional para centralizar no alvo vermelho."""
        if self.line_center[0] == -1:
            self.send_velocity(linear_x=0.0, angular_z=0.0)  # Para se não houver alvo
            return

        image_center_x = 320
        image_center_y = 240  # Assumindo altura de 480

        error_x = self.line_center[0] - image_center_x
        error_y = self.line_center[1] - image_center_y  # Controla para frente/trás

        # Se o erro for pequeno o suficiente, para.
        if abs(error_x) < self.CENTERING_TOLERANCE and abs(error_y) < self.CENTERING_TOLERANCE:
            self.send_velocity(linear_x=0.0, angular_z=0.0)
            return "ALINHADO"

        # Usa os erros para gerar comandos de velocidade
        linear_x = -self.KP_LINEAR * error_y  # Erro em Y controla velocidade para frente/trás
        angular_z = -self.KP_ANGULAR * error_x  # Erro em X controla rotação

        self.send_velocity(linear_x=linear_x, angular_z=angular_z)
        return "ALINHANDO"

    def go_to_home(self):
        """Controlador Proporcional para retornar à base."""
        if not self.current_pose or not self.home_position:
            return

        error_x = self.home_position.x - self.current_pose.position.x
        error_y = self.home_position.y - self.current_pose.position.y
        dist_to_home = math.sqrt(error_x ** 2 + error_y ** 2)

        if dist_to_home < 0.5:  # Tolerância de chegada
            self.send_velocity(0.0, 0.0)
            return "CHEGOU"

        # Calcula o ângulo para a base e o erro de orientação
        angle_to_home = math.atan2(error_y, error_x)
        current_yaw = self.quat_to_yaw(self.current_pose.orientation)
        error_angle = self.normalize_angle(angle_to_home - current_yaw)

        # Controle
        linear_x = 0.8 if dist_to_home > 2.0 else 0.4  # Reduz a velocidade perto de casa
        angular_z = 0.5 * error_angle

        self.send_velocity(linear_x, angular_z)
        return "NAVEGANDO"

    def send_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publica um comando de velocidade."""
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        self.velocity_pub.publish(vel_msg)

    def publish_gancho_centered(self):
        msg = String()
        msg.data = "Drone centralizado"
        self.gancho_pos_pub.publish(msg)
        self.get_logger().info("Publicado: 'Drone centralizado'")

    # --- Máquina de Estados da Missão (Loop Principal) ---
    async def mission_step(self):
        """Executa um passo da máquina de estados da missão."""
        if not self.current_fcu_state.connected or self.current_pose is None:
            self.get_logger().warn("Aguardando conexão com FCU e dados de pose...", throttle_duration_sec=5)
            return

        # --- ESTADO: INICIO ---
        if self.mission_state == "INICIO":
            self.get_logger().info("Estado da Missão: INICIO. Configurando modo para GUIDED e armando...")
            if self.current_fcu_state.mode != "GUIDED":
                await self.set_mode("GUIDED")
                return  # Aguarda próximo ciclo para verificar
            if not self.current_fcu_state.armed:
                await self.arm()
                return  # Aguarda próximo ciclo para verificar
            self.mission_state = "DECOLANDO"
            self.get_logger().info("Pronto para decolar.")

        # --- ESTADO: DECOLANDO ---
        elif self.mission_state == "DECOLANDO":
            self.get_logger().info(f"Estado da Missão: DECOLANDO para {self.ALTITUDE_CRUZEIRO}m")
            await self.takeoff(self.ALTITUDE_CRUZEIRO)
            self.mission_state = "AGUARDANDO_DECOLAGEM"

        elif self.mission_state == "AGUARDANDO_DECOLAGEM":
            if self.current_pose.position.z >= self.ALTITUDE_CRUZEIRO * 0.95:
                self.get_logger().info("Decolagem concluída. Aguardando 3s para estabilizar.")
                time.sleep(3)  # Pausa para estabilização
                self.mission_state = "SEGUINDO_LINHA"
            else:
                self.get_logger().info(f"Aguardando atingir altitude... Atual: {self.current_pose.position.z:.2f}m",
                                       throttle_duration_sec=2)

        # --- ESTADO: SEGUINDO_LINHA ---
        elif self.mission_state == "SEGUINDO_LINHA":
            self.get_logger().info("Estado da Missão: SEGUINDO_LINHA", throttle_duration_sec=5)
            if self.vision_state == "vermelho_detectado":
                self.get_logger().info("Alvo vermelho detectado! Mudando para estado CENTRALIZANDO.")
                self.send_velocity(0.0, 0.0)  # Para o drone
                time.sleep(1)  # Pausa antes de começar a centralizar
                self.mission_state = "CENTRALIZANDO"
            else:
                self.follow_line()

        # --- ESTADO: CENTRALIZANDO ---
        elif self.mission_state == "CENTRALIZANDO":
            self.get_logger().info("Estado da Missão: CENTRALIZANDO no alvo", throttle_duration_sec=3)
            status = self.center_on_target()
            if status == "ALINHADO":
                self.get_logger().info("Drone alinhado com o alvo.")
                self.publish_gancho_centered()
                self.mission_state = "SOLTANDO_GANCHO"

        # --- ESTADO: SOLTANDO_GANCHO ---
        elif self.mission_state == "SOLTANDO_GANCHO":
            self.get_logger().info(f"Estado da Missão: SOLTANDO_GANCHO. Status atual do gancho: {self.gancho_status}",
                                   throttle_duration_sec=2)
            if self.gancho_status == 'released':
                self.get_logger().info("Gancho solto com sucesso! Retornando à base.")
                time.sleep(2)  # Pausa antes de retornar
                self.mission_state = "VOLTANDO"
            else:
                # Re-publica a mensagem caso o nó do gancho não tenha recebido
                self.publish_gancho_centered()
                time.sleep(1)

        # --- ESTADO: VOLTANDO ---
        elif self.mission_state == "VOLTANDO":
            self.get_logger().info("Estado da Missão: VOLTANDO para a base.", throttle_duration_sec=5)
            status = self.go_to_home()
            if status == "CHEGOU":
                self.get_logger().info("Chegou à base. Preparando para pousar.")
                self.mission_state = "POUSANDO"

        # --- ESTADO: POUSANDO ---
        elif self.mission_state == "POUSANDO":
            self.get_logger().info("Estado da Missão: POUSANDO.")
            await self.set_mode("LAND")  # Usa o modo de pouso do MAVROS
            self.mission_state = "AGUARDANDO_POUSO"

        elif self.mission_state == "AGUARDANDO_POUSO":
            if not self.current_fcu_state.armed:
                self.get_logger().info("Pouso concluído e drone desarmado.")
                self.mission_state = "FIM"
            else:
                self.get_logger().info("Aguardando pouso e desarme...", throttle_duration_sec=3)

        # --- ESTADO: FIM ---
        elif self.mission_state == "FIM":
            self.get_logger().info("MISSÃO CONCLUÍDA!")
            self.mission_timer.cancel()  # Para o loop da missão
            rclpy.shutdown()

    # --- Funções Utilitárias ---
    def quat_to_yaw(self, q):
        """Converte um quaternion para ângulo yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normaliza um ângulo para o intervalo [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navegação interrompida pelo usuário.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()