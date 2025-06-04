#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo ROS 2: detector_personas_armas

- Se suscribe a un topic sensor_msgs/msg/Image (por defecto: "imagen").
- Cada <detection_interval> segundos toma el último frame recibido,
  lo envía a Roboflow y, si hay alguna detección, guarda el frame
  en la carpeta <save_directory>.

Parámetros ROS 2 (se pasan con --ros-args -p <param>=<valor>):
  image_topic            (string, por defecto "imagen")
  detection_interval     (float,  por defecto 3.0)
  confidence_threshold   (float,  por defecto 0.7)
  roboflow_api_key       (string, "Ql8QEkOdDeWs4MedxmCd")
  roboflow_project_slug  (string, por defecto "gun-detection")
  roboflow_project_version (int,  por defecto 1)
  save_directory         (string, por defecto "/tmp/detecciones")
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import threading
import time
import os


from roboflow import Roboflow


class DetectorPersonasArmas(Node):
    def __init__(self):
        super().__init__('detector_personas_armas')

        # ------------------------------------------------
        # 1) “Quemamos” la API Key, el slug y la versión
        # ------------------------------------------------
        self.api_key          = "Ql8QEkOdDeWs4MedxmCd"
        self.project_slug     = "border_wacthv2-dmbwq"
        self.project_version  = 1

        # ------------------------------------------------
        # 2) Parámetros ROS 2 “normales”
        # ------------------------------------------------
        self.declare_parameter('image_topic', 'image')
        self.declare_parameter('detection_interval', 3.0)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('save_directory', '/tmp/detecciones')

        # Recuperar valores
        self.image_topic        = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detection_interval = self.get_parameter('detection_interval').get_parameter_value().double_value
        self.conf_threshold     = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.save_directory     = self.get_parameter('save_directory').get_parameter_value().string_value

        # ------------------------------
        # 2) VALIDAR API KEY Y CREAR CARPETA
        # ------------------------------
        if not self.api_key:
            self.get_logger().error("¡No has definido 'roboflow_api_key'! El nodo se detiene.")
            rclpy.shutdown()
            return

        try:
            os.makedirs(self.save_directory, exist_ok=True)
            self.get_logger().info(f"Carpeta de guardado: {self.save_directory}")
        except Exception as e:
            self.get_logger().error(f"No se pudo crear carpeta '{self.save_directory}': {e}")
            rclpy.shutdown()
            return

        # ------------------------------
        # 3) VARIABLES INTERNAS
        # ------------------------------
        self.bridge = CvBridge()
        self.ultimo_frame = None
        self._frame_lock = threading.Lock()

        # ------------------------------
        # 4) INICIALIZAR ROBOFLOW
        # ------------------------------
        try:
            rf = Roboflow(api_key=self.api_key)
            project = rf.workspace().project(self.project_slug)
            self.model = project.version(self.project_version).model
            self.get_logger().info(
                f"Roboflow: modelo '{self.project_slug}', versión {self.project_version} cargado."
            )
        except Exception as e:
            self.get_logger().error(f"Error inicializando Roboflow: {e}")
            rclpy.shutdown()
            return

        # ------------------------------
        # X) QoSProfile “tipo sensor” (depth=1, best_effort)
        # ------------------------------
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        # ------------------------------
        # 5) SUSCRIBIRSE AL TOPIC DE IMAGEN
        # ------------------------------
        self.create_subscription(
            Image,
            self.image_topic,
            self._imagen_callback,
            qos_profile=qos
        )
        self.get_logger().info(f"Suscrito al topic Image (QoS best_effort): '{self.image_topic}'")
        # ------------------------------
        # 6) CREAR TIMER DE DETECCIÓN
        # ------------------------------
        self.create_timer(self.detection_interval, self._timer_callback)
        self.get_logger().info(
            f"Timer creado: detectando cada {self.detection_interval:.1f} s con confianza >= {self.conf_threshold:.2f}"
        )

    def _imagen_callback(self, msg: Image):
        """
        Callback al recibir un sensor_msgs/Image.
        Convertimos a OpenCV BGR y guardamos el frame en self.ultimo_frame.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f"Error en cv_bridge: {e}")
            return

        with self._frame_lock:
            self.ultimo_frame = cv_image.copy()
            # Log para confirmar que llegó un frame
            self.get_logger().info("► He recibido y almacenado un frame.")

    def _timer_callback(self):
        """
        Cada <detection_interval> segundos se ejecuta este callback.
        Toma el último frame y llama a detectar_personas_armas().
        """
        with self._frame_lock:
            if self.ultimo_frame is None:
                self.get_logger().warn("Aún no ha llegado ningún frame. Esperando...")
                return
            frame = self.ultimo_frame.copy()

        self.detectar_personas_armas(frame)

    def detectar_personas_armas(self, frame_bgr: np.ndarray):
        """
        Envía el frame a Roboflow y, si hay detecciones, guarda el frame completo
        en la carpeta save_directory con nombre basado en timestamp.
        """
        t0 = time.time()
        self.get_logger().info(f"▶ Inferencia Roboflow t={t0:.2f}")

        # 1) Guardar temporalmente
        temp_path = '/tmp/robo_frame_ros2.png'
        cv2.imwrite(temp_path, frame_bgr)

        # 2) Predecir en Roboflow
        try:
            resultados = self.model.predict(temp_path, confidence=self.conf_threshold).json()
        except Exception as e:
            self.get_logger().error(f"Error al predecir con Roboflow: {e}")
            return

        # 3) Revisar si hay detecciones
        detecciones = resultados.get('predictions', [])
        if len(detecciones) == 0:
            self.get_logger().info("– Sin detecciones en este frame.")
        else:
            num = len(detecciones)
            self.get_logger().info(f"✔️ Se detectaron {num} objeto(s). Guardando frame...")

            # 4) Guardar el frame con nombre timestamp
            timestamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
            filename = f"deteccion_{timestamp}.png"
            full_path = os.path.join(self.save_directory, filename)

            try:
                cv2.imwrite(full_path, frame_bgr)
                self.get_logger().info(f"Frame guardado en: {full_path}")
            except Exception as e:
                self.get_logger().error(f"No se pudo guardar frame en '{full_path}': {e}")

        t1 = time.time()
        self.get_logger().info(f"⏱ Inferencia completada en {(t1 - t0):.2f} s")

    def destroy_node(self):
        """
        Al destruir el nodo, cerramos ventanas de OpenCV (aunque no se abran).
        """
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    nodo = DetectorPersonasArmas()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.get_logger().info("Cerrando nodo detector_personas_armas...")
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
