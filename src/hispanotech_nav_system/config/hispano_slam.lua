--- @file HispanoTech_slam.lua
--- @brief Configuración de SLAM 2D para HispanoTech usando Cartographer
--- Este archivo contiene la configuración personalizada para el sistema de mapeo y localización
--- de HispanoTech. Configura parámetros clave para:
--- - Integración con sensores específicos del robot
--- - Estrategias de optimización de mapas
--- - Publicación de datos de navegación

-- Módulos principales de Cartographer
include "map_builder.lua"
include "trajectory_builder.lua"

--- @table options
--- @brief Configuración principal del sistema SLAM HispanoTech
--- @field map_builder Configuración del constructor de mapas (MapBuilder)
--- @field trajectory_builder Configuración de construcción de trayectoria
--- @field map_frame Frame del mapa global (sistema de coordenadas fijo)
--- @field tracking_frame Frame del sensor primario de tracking (IMU)
--- @field published_frame Frame base publicado para integración con ROS
--- @field use_odometry Indica si se usa odometría wheel-based
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",         
  published_frame = "odom",        
  odom_frame = "odom",
  provide_odom_frame = false,            
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--- Configuración específica HispanoTech para mapeo 2D
--- @remark Optimizado para entornos industriales medianos
MAP_BUILDER.use_trajectory_builder_2d = true

--- @table TRAJECTORY_BUILDER_2D
--- @brief Parámetros del lidar LMS-200 (HispanoTech Modelo 2024)
--- @field min_range Rango mínimo operativo del sensor
--- @field max_range Rango máximo efectivo del sensor
--- @field missing_data_ray_length Manejo de obstáculos no detectados
--- @field use_online_correlative_scan_matching Matching correlativo para precisión
TRAJECTORY_BUILDER_2D.min_range = 0.12  -- 12 cm mínimo
TRAJECTORY_BUILDER_2D.max_range = 3.5    -- 3.5 metros máximo
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Configuración sin IMU
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

--- Políticas de optimización HispanoTech
--- @table POSE_GRAPH
--- @field constraint_builder Ajustes para restricciones de loop closure
--- @field global_localization_min_score Umbral para relocalización global
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 65% confianza mínima
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 70% para relocalización

-- POSE_GRAPH.optimize_every_n_nodes = 0  -- Optimización continua desactivada

--- @return table Configuración completa del sistema SLAM HispanoTech
return options