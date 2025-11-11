#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class NavigatorNode(Node):
    """
    Nó principal de navegação (ROS 2) que integra visão e gancho para a missão.
    Implementa uma máquina de estados não-bloqueante com chamadas de serviço assíncronas.
    """

    def __init__(self):
        super().__init__("navigator_node")

        # ======================
        # Parâmetros configuráveis
        # ======================
        self.declare_parameter("altitude_cruzeiro", 3.0)
        self.declare_parameter("ganho_p_linear", 0.005)
        self.declare_parameter("ganho_p_angular", 0.002)
        self.declare_parameter("tolerancia_centralizacao", 25)
        self.declare_parameter("image_center_x", 320)
        self.declare_parameter("image_center_y", 240)

        self.ALTITUDE_CRUZEIRO = float(self.get_parameter("altitude_cruzeiro").value)
        self.KP_LINEAR = float(self.get_parameter("ganho_p_linear").value)
        self.KP_ANGULAR = float(self.get_parameter("ganho_p_angular").value)
        self.CENTER_TOL = int(self.get_parameter("tolerancia_centralizacao").value)
        self.IMG_CX = int(self.get_parameter("image_center_x").value)
        self.IMG_CY = int(self.get_parameter("image_center_y").value)

        # ======================
        # Estado interno
        # ======================
        # Máquina de estados
        self.state = "INICIO"  # INICIO, DECOLANDO, AGUARDA_DECOLAGEM, SEGUINDO, CENTRALIZANDO, SOLTANDO, VOLTANDO, POUSANDO, AGUARDA_POUSO, FIM

        # MAV / pose
        self.fcu_state = State()
        self.pose: Optional[PoseStamped] = None
        self.home = None  # (x, y) ao receber a primeira pose

        # Visão
        self.vision_state = "perdido"
        self.line_x = -1
        self.line_y = -1

        # Gancho
        self.gancho_status = "waiting"

        # Controle de chamadas assíncronas
        self.pending_future = None
        self.pending_kind = None  # "mode" | "arm" | "takeoff" | "land"

        # Auxiliares de tempo/estabilização
        self.t0 = self.get_clock().now()
        self.last_info_log = self.get_clock().now()
        self.steady_aligned_count = 0
        self.last_gancho_pub = self.get_clock().now()

        # ======================
        # ROS wiring
        # ======================
        self.cb_group = ReentrantCallbackGroup()
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_vel = self.create_publisher(Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10)
        self.pub_gancho_pos = self.create_publisher(String, "/gancho/posicao_drone", 10)

        # Subscribers
        self.sub_state = self.create_subscription(State, "/mavros/state", self._on_state, qos_be)
        self.sub_pose = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pose, qos_be)
        self.sub_vision_state = self.create_subscription(String, "/vision/state", self._on_vision_state, 10)
        self.sub_line = self.create_subscription(Int32MultiArray, "/vision/line_position", self._on_line_position, 10)
        self.sub_gancho = self.create_subscription(String, "/gancho/status", self._on_gancho_status, 10)

        # Service clients
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")

        # Timer principal (10 Hz)
        self.timer = self.create_timer(0.1, self._step, callback_group=self.cb_group)

        self.get_logger().info(f"Nó de navegação iniciado. Altitude de cruzeiro: {self.ALTITUDE_CRUZEIRO:.2f} m")

    # ======================
    # Callbacks de assinatura
    # ======================
    def _on_state(self, msg: State):
        self.fcu_state = msg

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg
        if self.home is None:
            self.home = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(f"HOME definido: x={self.home[0]:.2f}, y={self.home[1]:.2f}")

    def _on_vision_state(self, msg: String):
        self.vision_state = msg.data

    def _on_line_position(self, msg: Int32MultiArray):
        # Espera [x, y]
        if len(msg.data) >= 2:
            self.line_x = int(msg.data[0])
            self.line_y = int(msg.data[1])
        else:
            self.line_x = -1
            self.line_y = -1

    def _on_gancho_status(self, msg: String):
        self.gancho_status = msg.data

    # ======================
    # Helpers: comandos e math
    # ======================
    def _send_vel(self, vx: float = 0.0, wz: float = 0.0):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.pub_vel.publish(cmd)

    @staticmethod
    def _quat_to_yaw(q):
        # z-yaw only
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _norm_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ======================
    # Helpers: serviços assíncronos (não-bloqueantes)
    # ======================
    def _request_set_mode(self, mode: str):
        if not self.cli_mode.wait_for_service(timeout_sec=0.0):
            self._log_every(2.0, "Aguardando serviço /mavros/set_mode...")
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        self.pending_future = self.cli_mode.call_async(req)
        self.pending_kind = "mode"
        self.get_logger().info(f"Solicitado modo: {mode}")
        return True

    def _request_arm(self, value: bool):
        if not self.cli_arm.wait_for_service(timeout_sec=0.0):
            self._log_every(2.0, "Aguardando serviço /mavros/cmd/arming...")
            return False
        req = CommandBool.Request()
        req.value = bool(value)
        self.pending_future = self.cli_arm.call_async(req)
        self.pending_kind = "arm"
        self.get_logger().info(f"Solicitado armar={value}")
        return True

    def _request_takeoff(self, alt: float):
        if not self.cli_takeoff.wait_for_service(timeout_sec=0.0):
            self._log_every(2.0, "Aguardando serviço /mavros/cmd/takeoff...")
            return False
        req = CommandTOL.Request()
        req.altitude = float(alt)
        self.pending_future = self.cli_takeoff.call_async(req)
        self.pending_kind = "takeoff"
        self.get_logger().info(f"Solicitado takeoff para {alt:.2f} m")
        return True

    def _request_land(self):
        if not self.cli_land.wait_for_service(timeout_sec=0.0):
            self._log_every(2.0, "Aguardando serviço /mavros/cmd/land...")
            return False
        req = CommandTOL.Request()
        self.pending_future = self.cli_land.call_async(req)
        self.pending_kind = "land"
        self.get_logger().info("Solicitado land")
        return True

    def _handle_pending(self) -> bool:
        """
        Verifica se há chamada assíncrona pendente. Se ainda não terminou, retorna True (aguardando).
        Se terminou, processa o resultado e libera o pending_*.
        Retorna False quando não há nada pendente.
        """
        if self.pending_future is None:
            return False
        if not self.pending_future.done():
            return True

        try:
            res = self.pending_future.result()
        except Exception as e:
            self.get_logger().error(f"Erro na chamada '{self.pending_kind}': {e}")
            self.pending_future = None
            self.pending_kind = None
            return False

        kind = self.pending_kind
        self.pending_future = None
        self.pending_kind = None

        if kind == "mode":
            if getattr(res, "mode_sent", False):
                self.get_logger().info("Modo alterado com sucesso.")
            else:
                self.get_logger().error("Falha ao alterar modo.")
        elif kind == "arm":
            if getattr(res, "success", False):
                self.get_logger().info("Armar/desarmar concluído.")
            else:
                self.get_logger().error("Falha no armar/desarmar.")
        elif kind == "takeoff":
            if getattr(res, "success", False):
                self.get_logger().info("Takeoff aceito.")
            else:
                self.get_logger().error("Falha no takeoff.")
        elif kind == "land":
            if getattr(res, "success", False):
                self.get_logger().info("Land aceito.")
            else:
                self.get_logger().error("Falha no land.")

        return False

    # ======================
    # Controle de linha e centralização
    # ======================
    def _follow_line(self):
        if self.line_x < 0 or self.vision_state == "perdido":
            # Procura linha: gira devagar
            self._send_vel(0.0, 0.3)
            return

        error_x = self.line_x - self.IMG_CX
        vx = 0.5
        wz = -self.KP_ANGULAR * float(error_x)
        self._send_vel(vx, wz)

    def _center_on_target(self) -> bool:
        """
        Usa posição [x, y] para centralizar.
        Retorna True quando está alinhado de forma estável por múltiplos ciclos.
        """
        if self.line_x < 0 or self.line_y < 0:
            self._send_vel(0.0, 0.0)
            self.steady_aligned_count = 0
            return False

        err_x = self.line_x - self.IMG_CX
        err_y = self.line_y - self.IMG_CY

        if abs(err_x) <= self.CENTER_TOL and abs(err_y) <= self.CENTER_TOL:
            # Conta alguns ciclos estáveis para evitar falsos positivos
            self.steady_aligned_count += 1
            self._send_vel(0.0, 0.0)
            return self.steady_aligned_count >= 5  # ~0.5s em 10 Hz
        else:
            self.steady_aligned_count = 0

        vx = -self.KP_LINEAR * float(err_y)  # y da imagem -> frente/trás
        wz = -self.KP_ANGULAR * float(err_x)  # x da imagem -> yaw
        self._send_vel(vx, wz)
        return False

    def _go_home(self) -> bool:
        """
        Controla retorno ao ponto HOME.
        Retorna True quando chegou (dist < 0.5 m).
        """
        if self.pose is None or self.home is None:
            return False

        curr = self.pose.pose
        dx = float(self.home[0] - curr.position.x)
        dy = float(self.home[1] - curr.position.y)
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            self._send_vel(0.0, 0.0)
            return True

        heading = math.atan2(dy, dx)
        yaw = self._quat_to_yaw(curr.orientation)
        err_yaw = self._norm_angle(heading - yaw)

        vx = 0.8 if dist > 2.0 else 0.4
        wz = 0.5 * err_yaw
        self._send_vel(vx, wz)
        return False

    def _publish_gancho_centralizado(self):
        now = self.get_clock().now()
        if (now - self.last_gancho_pub).nanoseconds * 1e-9 < 0.5:
            return
        self.last_gancho_pub = now
        msg = String()
        msg.data = "Drone centralizado"
        self.pub_gancho_pos.publish(msg)
        self.get_logger().info("Publicado: 'Drone centralizado'")

    def _log_every(self, seconds: float, msg: str):
        now = self.get_clock().now()
        elapsed = (now - self.last_info_log).nanoseconds * 1e-9
        if elapsed >= seconds:
            self.last_info_log = now
            self.get_logger().info(msg)

    # ======================
    # Loop principal da missão
    # ======================
    def _step(self):
        # Pré-condições
        if not getattr(self.fcu_state, "connected", False) or self.pose is None:
            self._log_every(3.0, "Aguardando conexão com FCU e pose...")
            return

        # Se há uma chamada de serviço pendente, aguarde concluir
        if self._handle_pending():
            return

        # Máquina de estados
        if self.state == "INICIO":
            # Modo (ArduPilot): GUIDED. (Para PX4 seria OFFBOARD.)
            if self.fcu_state.mode != "GUIDED":
                if self._request_set_mode("GUIDED"):
                    return  # aguarda resposta
            elif not self.fcu_state.armed:
                if self._request_arm(True):
                    return
            else:
                self.state = "DECOLANDO"
                self.t0 = self.get_clock().now()
                self.get_logger().info("Transição -> DECOLANDO")

        elif self.state == "DECOLANDO":
            if self._request_takeoff(self.ALTITUDE_CRUZEIRO):
                self.state = "AGUARDA_DECOLAGEM"
                self.get_logger().info("Transição -> AGUARDA_DECOLAGEM")

        elif self.state == "AGUARDA_DECOLAGEM":
            alt = float(self.pose.pose.position.z) if self.pose else 0.0
            if alt >= 0.95 * self.ALTITUDE_CRUZEIRO:
                self.state = "SEGUINDO"
                self.get_logger().info("Decolagem concluída. Transição -> SEGUINDO")
            else:
                self._log_every(2.0, f"Aguardando atingir altitude... atual={alt:.2f} m")

        elif self.state == "SEGUINDO":
            # Se visão sinalizar vermelho, interrompe e centraliza
            if self.vision_state == "vermelho_detectado":
                self._send_vel(0.0, 0.0)
                self.state = "CENTRALIZANDO"
                self.get_logger().info("Alvo vermelho detectado. Transição -> CENTRALIZANDO")
            else:
                self._follow_line()

        elif self.state == "CENTRALIZANDO":
            aligned = self._center_on_target()
            if aligned:
                self._publish_gancho_centralizado()
                self.state = "SOLTANDO"
                self.get_logger().info("Alinhado. Transição -> SOLTANDO")

        elif self.state == "SOLTANDO":
            if self.gancho_status == "released":
                self.state = "VOLTANDO"
                self.get_logger().info("Gancho liberado. Transição -> VOLTANDO")
            else:
                # Reenvia o sinal periodicamente
                self._publish_gancho_centralizado()

        elif self.state == "VOLTANDO":
            if self._go_home():
                self.state = "POUSANDO"
                self.get_logger().info("Chegou ao HOME. Transição -> POUSANDO")

        elif self.state == "POUSANDO":
            # Preferimos comando LAND para fechar missão e desarmar
            if self._request_land():
                self.state = "AGUARDA_POUSO"
                self.get_logger().info("Transição -> AGUARDA_POUSO")

        elif self.state == "AGUARDA_POUSO":
            if not self.fcu_state.armed:
                self.state = "FIM"
                self.get_logger().info("Pouso concluído. Transição -> FIM")
            else:
                self._log_every(3.0, "Aguardando pouso/desarme...")

        elif self.state == "FIM":
            self._send_vel(0.0, 0.0)
            self._log_every(5.0, "MISSÃO CONCLUÍDA.")
            # Não desligamos o nó aqui para facilitar re-observação dos tópicos.


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()