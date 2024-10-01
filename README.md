# joy_franka_vel_control

Este paquete de ROS se ha desarrollado para teleoperar el controlador de velocidad cartesiana del manipulador Franka Emika Panda mediante un mando linux. La creación de este paquete se enmarca dentro del proyecto RAFI.

## Funcionamiento
Este paquete contiene el nodo `joy_franka_node`. Este nodo se subscribe al topic `/Joy` para obtener el estado del mando en forma de mensaje `sensor_msg/joy`. Usa esa información para mandar comandos de velocidad al controlador del manipulador. Para ello, publica un mensaje de `Geometry_msg/Twist` en el controlador.

Se utiliza una versión modificada del controlador de ejemplo de velocidad cartesiana de `franka_example_controllers`. Esta versión permite la subscripción a un topic para obtener la velocidad deseada.

## Instalación

### Paquetes necesarios
Para su uso, se debe tener instalados los siguientes paquetes. Se adjunta el enlace que contiene información sobre su instalación.
- `joy`. Driver para el mando. Disponible en: http://wiki.ros.org/joy.
- `franka_ros_rafi`. Se trata de una versión modificada del metapaquete `franka_ros`. <!-- añadir enlace al metapaquete modificado. lo que se ha modificado  -->

<!-- Se debe tener instalado y configurado el metapaquete `franka_ros` así como la librería `libfranka`. Disponible en: https://frankaemika.github.io/docs/installation_linux.html. -->

### Instalación de este paquete

```bash
git clone https://github.com/rodri-castro/joy_franka_vel_control.git
catkin_make
```

## Uso

```bash
roslaunch joy_franka_vel_control teleop_franka.launch
```
Este comando lanza el nodo del driver del mando junto con el nodo `joy_franka_node`. Este archivo de lanzamiento no lanza el controlador de velocidad cartesiana del manipulador.

