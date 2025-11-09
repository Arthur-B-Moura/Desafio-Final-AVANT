"""
line_detector.py
Nó ROS2 em Python para:
- detectar e seguir linha azul no chão;
- detectar mangueira/trecho vermelho;
- publicar centro da linha e sinal de vermelho detectado;
- publicar imagem de debug anotada.

Publicações:
- /vision/line_position: Int32MultiArray [cx, cy] (pixels). [-1,-1] quando não detectado.
- /vision/red_detected : Bool
- /vision/state : String ("seguindo", "vermelho_detectado", "perdido")
- /vision/debug_image : sensor_msgs/Image (opcional para visualização)
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Bool, String
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from typing import Tuple, Deque


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        
        # Parâmetros (ajustáveis)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug_image', True)
        
        # Faixa azul (HSV)
        self.declare_parameter('azul_h_min', 100)
        self.declare_parameter('azul_s_min', 150)
        self.declare_parameter('azul_v_min', 50)
        self.declare_parameter('azul_h_max', 140)
        self.declare_parameter('azul_s_max', 255)
        self.declare_parameter('azul_v_max', 255)
        
        # Faixa vermelho (duas faixas por causa do wrap em H)
        self.declare_parameter('red1_h_min', 0)
        self.declare_parameter('red1_s_min', 120)
        self.declare_parameter('red1_v_min', 70)
        self.declare_parameter('red1_h_max', 10)
        self.declare_parameter('red1_s_max', 255)
        self.declare_parameter('red1_v_max', 255)
        
        self.declare_parameter('red2_h_min', 170)
        self.declare_parameter('red2_s_min', 120)
        self.declare_parameter('red2_v_min', 70)
        self.declare_parameter('red2_h_max', 180)
        self.declare_parameter('red2_s_max', 255)
        self.declare_parameter('red2_v_max', 255)
        
        # Fila de suavização (média móvel)
        self.declare_parameter('smoothing_window', 5)
        
        # Mínima área do contorno em pixels para considerar detecção válida
        self.declare_parameter('min_area', 300)
        
        # Fração vertical inferior da imagem usada para detectar a linha (0.0..1.0)
        self.declare_parameter('roi_vertical_fraction', 0.4)
        
        # Operação morfológica
        self.declare_parameter('morph_kernel_size', 5)
        
        # Modo calibração (opcional)
        self.declare_parameter('calibration_mode', False)
        
        # Carrega parâmetros
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        
        # Carrega faixas azul
        self.azul_lower = np.array([
            self.get_parameter('azul_h_min').value,
            self.get_parameter('azul_s_min').value,
            self.get_parameter('azul_v_min').value,
        ])
        self.azul_upper = np.array([
            self.get_parameter('azul_h_max').value,
            self.get_parameter('azul_s_max').value,
            self.get_parameter('azul_v_max').value,
        ])
        
        # Carrega faixas vermelho
        self.red1_lower = np.array([
            self.get_parameter('red1_h_min').value,
            self.get_parameter('red1_s_min').value,
            self.get_parameter('red1_v_min').value,
        ])
        self.red1_upper = np.array([
            self.get_parameter('red1_h_max').value,
            self.get_parameter('red1_s_max').value,
            self.get_parameter('red1_v_max').value,
        ])
        self.red2_lower = np.array([
            self.get_parameter('red2_h_min').value,
            self.get_parameter('red2_s_min').value,
            self.get_parameter('red2_v_min').value,
        ])
        self.red2_upper = np.array([
            self.get_parameter('red2_h_max').value,
            self.get_parameter('red2_s_max').value,
            self.get_parameter('red2_v_max').value,
        ])
        
        self.smoothing_window = self.get_parameter('smoothing_window').get_parameter_value().integer_value
        self.min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.roi_vertical_fraction = self.get_parameter('roi_vertical_fraction').get_parameter_value().double_value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').get_parameter_value().integer_value
        self.calibration_mode = self.get_parameter('calibration_mode').get_parameter_value().bool_value
        
        # Validação de parâmetros
        if not (0.0 <= self.roi_vertical_fraction <= 1.0):
            self.get_logger().warn(
                f'roi_vertical_fraction={self.roi_vertical_fraction} fora do intervalo [0,1], usando 0.4'
            )
            self.roi_vertical_fraction = 0.4
        
        # Publishers
        self.pub_line_position = self.create_publisher(Int32MultiArray, '/vision/line_position', 10)
        self.pub_red_detected = self.create_publisher(Bool, '/vision/red_detected', 10)
        self.pub_state = self.create_publisher(String, '/vision/state', 10)
        
        if self.publish_debug_image:
            self.pub_debug_image = self.create_publisher(Image, '/vision/debug_image', 10)
        
        # Subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'Assinando tópico de câmera: {self.camera_topic}')
        
        # Filas para suavizar centros (média móvel)
        self.center_history_blue : Deque[Tuple[int, int]] = deque(maxlen=self.smoothing_window)
        self.center_history_red  : Deque[Tuple[int, int]] = deque(maxlen=self.smoothing_window)
        
        # Estado inicial
        self.current_state = 'perdido' # "seguindo", "vermelho_detectado", "perdido"
        self.lost_frames_counter = 0   # Contador de frames perdidos consecutivos

    def image_callback(self, msg: Image) -> None:
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
            return
        
        try:
            frame_h, frame_w = frame_bgr.shape[:2]
            
            # ROI: parte inferior da imagem (onde está a linha no chão)
            y_start = int(frame_h * (1.0 - self.roi_vertical_fraction))
            roi = frame_bgr[y_start:frame_h, 0:frame_w].copy()
            
            # Processa HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Máscaras
            mask_azul = cv2.inRange(hsv, self.azul_lower, self.azul_upper)
            mask_red_1 = cv2.inRange(hsv, self.red1_lower, self.red1_upper)
            mask_red_2 = cv2.inRange(hsv, self.red2_lower, self.red2_upper)
            mask_vermelho = cv2.bitwise_or(mask_red_1, mask_red_2)
            
            # Morfologia para reduzir ruído
            kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, kernel)
            mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, kernel)
            mask_vermelho = cv2.morphologyEx(mask_vermelho, cv2.MORPH_OPEN, kernel)
            mask_vermelho = cv2.morphologyEx(mask_vermelho, cv2.MORPH_CLOSE, kernel)
            
            # Modo calibração: mostra máscaras
            if self.calibration_mode:
                cv2.imshow('Mask Azul', mask_azul)
                cv2.imshow('Mask Vermelho', mask_vermelho)
                cv2.imshow('ROI Original', roi)
                cv2.waitKey(1)
            
            # Detectar vermelho (prioritário)
            red_found, red_center = self._find_largest_contour_center(
                mask_vermelho,
                min_area=self.min_area
            )
            
            if red_found:
                # Limpa histórico da linha azul ao trocar de estado
                self.center_history_blue.clear()
                
                # Transforma em coordenadas da imagem completa
                red_cx_rel, red_cy_rel = red_center
                red_cx_global = red_cx_rel
                red_cy_global = red_cy_rel + y_start
                
                # Suaviza centro vermelho
                smoothed_red = self._smooth_center_red((red_cx_global, red_cy_global))
                
                self._publish_red_detected(True)
                self._publish_state('vermelho_detectado')
                self._publish_line_position(int(smoothed_red[0]), int(smoothed_red[1]))
                
                # Reset contador de perdidos
                self.lost_frames_counter = 0
                
                # Debug image
                if self.publish_debug_image:
                    debug_frame = frame_bgr.copy()
                    cv2.circle(
                        debug_frame, 
                        (int(smoothed_red[0]), int(smoothed_red[1])), 
                        8, 
                        (0, 0, 255), 
                        -1
                    )

                    cv2.putText(
                        debug_frame,
                        f'VERMELHO ({int(smoothed_red[0])}, {int(smoothed_red[1])})',
                        (int(smoothed_red[0]) - 80, int(smoothed_red[1]) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2
                    )

                    cv2.rectangle(
                        debug_frame, 
                        (0, y_start), 
                        (frame_w - 1, frame_h - 1), 
                        (0, 255, 0), 
                        2
                    )
                    self._publish_debug_image(debug_frame, msg.header)
                return
            
            # Se não encontrou vermelho, tenta detectar a linha azul
            line_found, line_center = self._find_largest_contour_center(
                mask_azul,
                min_area=self.min_area
            )
            
            if line_found:
                # Limpa histórico vermelho ao trocar de estado
                self.center_history_red.clear()
                
                cx_rel, cy_rel = line_center  # relativo ao ROI
                cx_global = int(cx_rel)
                cy_global = int(cy_rel + y_start)
                
                # Suaviza com média móvel
                smoothed_center = self._smooth_center_blue((cx_global, cy_global))
                
                self._publish_line_position(int(smoothed_center[0]), int(smoothed_center[1]))
                self._publish_red_detected(False)
                self._publish_state('seguindo')
                
                # Reset contador de perdidos
                self.lost_frames_counter = 0
                
                # Debug image
                if self.publish_debug_image:
                    debug_frame = frame_bgr.copy()
                    cv2.circle(
                        debug_frame, 
                        (int(smoothed_center[0]), int(smoothed_center[1])), 
                        6, 
                        (255, 0, 0),
                        -1
                    )

                    cv2.putText(
                        debug_frame,
                        f'LINE cx={int(smoothed_center[0])} cy={int(smoothed_center[1])}',
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 0, 0),
                        2
                    )

                    cv2.rectangle(
                        debug_frame, 
                        (0, y_start), 
                        (frame_w - 1, frame_h - 1), 
                        (0, 255, 0), 
                        2
                    )
                    self._publish_debug_image(debug_frame, msg.header)
                return
            
            # Se nada foi encontrado
            self.lost_frames_counter += 1
            self._publish_line_position(-1, -1)
            self._publish_red_detected(False)
            self._publish_state('perdido')
            
            # Limpa históricos quando perdido
            if self.lost_frames_counter > 10:
                self.center_history_blue.clear()
                self.center_history_red.clear()
            
            if self.publish_debug_image:
                debug_frame = frame_bgr.copy()
                cv2.putText(
                    debug_frame,
                    f'PERDIDO (frames: {self.lost_frames_counter})',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )
                self._publish_debug_image(debug_frame, msg.header)
        
        except Exception as e:
            self.get_logger().error(f'Erro no processamento da imagem: {e}')

    def _find_largest_contour_center(
        self,
        mask: np.ndarray,
        min_area: int = 100
    ) -> Tuple[bool, Tuple[int, int]]:
        """
        Encontra o maior contorno na máscara e retorna (found, (cx, cy)) relativo à máscara.
        """
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, (-1, -1)
        
        # Pega maior contorno por área
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < min_area:
            return False, (-1, -1)
        
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return False, (-1, -1)
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        return True, (cx, cy)

    def _smooth_center_blue(
        self, 
        center: Tuple[int, int]
    ) -> Tuple[float, float]:
        """Suaviza centros da linha azul com média móvel."""
        self.center_history_blue.append(center)

        sx = sum([c[0] for c in self.center_history_blue]) / len(self.center_history_blue)
        sy = sum([c[1] for c in self.center_history_blue]) / len(self.center_history_blue)

        return sx, sy

    def _smooth_center_red(
        self,
        center: Tuple[int, int]
    ) -> Tuple[float, float]:
        """Suaviza centros do vermelho com média móvel."""
        self.center_history_red.append(center)

        sx = sum([c[0] for c in self.center_history_red]) / len(self.center_history_red)
        sy = sum([c[1] for c in self.center_history_red]) / len(self.center_history_red)

        return sx, sy

    def _publish_line_position(self, cx: int, cy: int) -> None:
        msg = Int32MultiArray()
        msg.data = [int(cx), int(cy)]
        self.pub_line_position.publish(msg)
        self.get_logger().debug(f'Publicado line_position: {msg.data}')

    def _publish_red_detected(self, detected: bool) -> None:
        msg = Bool()
        msg.data = bool(detected)
        self.pub_red_detected.publish(msg)
        self.get_logger().debug(f'Publicado red_detected: {detected}')

    def _publish_state(self, state_str: str) -> None:
        if state_str != self.current_state:
            # Log apenas quando há mudança de estado
            self.get_logger().info(f'State -> {state_str}')
            self.current_state = state_str
        
        msg = String()
        msg.data = state_str
        self.pub_state.publish(msg)

    def _publish_debug_image(self, frame_bgr: np.ndarray, header) -> None:
        """
        Publica imagem BGR anotada convertida para sensor_msgs/Image.
        """
        try:
            msg_img = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
            if header is not None:
                msg_img.header = header
            self.pub_debug_image.publish(msg_img)
        except CvBridgeError as e:
            self.get_logger().error(f'Erro ao converter debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Line detector interrompido.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()