"""Lanza un gait_recorder_node por camara ZED.

- camera_serials: lista de series separadas por comas; vacio = autodeteccion
  de hasta 2 camaras. Si un serial pedido no esta conectado, el nodo se lanza
  igualmente y reintenta la apertura (arranque degradado, ADR-002).
- svo_path: fuerza el modo simulacion con un unico nodo, ignorando las
  camaras reales.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAX_CAMERAS = 2


def get_connected_serials():
    # Import perezoso de pyzed: no hace falta en modo SVO y puede no estar
    # instalado en el python del sistema (ros2 launch usa /usr/bin/python3).
    try:
        import pyzed.sl as sl
        return [device.serial_number for device in sl.Camera.get_device_list()]
    except Exception as exc:  # pyzed sin SDK/camaras: se informa y se sigue
        print(f'[launcher] No se pudo enumerar camaras ZED: {exc}')
        return []


def launch_setup(context, *args, **kwargs):
    svo_path = LaunchConfiguration('svo_path').perform(context)
    serials_arg = LaunchConfiguration('camera_serials').perform(context).strip()
    config_file = LaunchConfiguration('config_file').perform(context)

    def make_node(serial, alias):
        return Node(
            package='zed_gait_analysis_recorder',
            executable='gait_recorder_node',
            name='gait_recorder_node',
            namespace=alias,
            output='screen',
            parameters=[config_file, {
                'camera_serial': serial,
                'camera_alias': alias,
                'svo_path': svo_path,
            }],
        )

    # Modo simulacion: un unico nodo con el SVO indicado.
    if svo_path:
        print(f'[launcher] Modo SVO forzado. Reproduciendo: {svo_path}')
        return [make_node(0, 'zed_svo')]

    detected = get_connected_serials()
    print(f'[launcher] Camaras detectadas: {detected or "ninguna"}')

    # Seleccion por serial (ADR-014).
    if serials_arg:
        nodes = []
        for token in serials_arg.replace(',', ' ').split():
            serial = int(token)
            if serial not in detected:
                print(f'[launcher] AVISO: la camara {serial} no esta '
                      'conectada; el nodo reintentara la apertura.')
            nodes.append(make_node(serial, f'zed{serial}'))
        return nodes

    if not detected:
        print('[launcher] ERROR: no se detecto ninguna camara y no se '
              'indicaron seriales. No se lanza ningun nodo.')
        return []

    return [make_node(serial, f'zed{serial}')
            for serial in detected[:MAX_CAMERAS]]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('zed_gait_analysis_recorder'),
        'config', 'default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_serials',
            default_value='',
            description='Series de camara separados por comas; vacio = '
                        'autodeteccion de hasta 2 camaras.'),
        DeclareLaunchArgument(
            'svo_path',
            default_value='',
            description='Ruta a un fichero SVO. Si se indica, se lanza un '
                        'unico nodo en modo simulacion.'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Fichero YAML de parametros.'),
        OpaqueFunction(function=launch_setup),
    ])
