/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include <map>
#include <string>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <franka/rate_limiting.h>
#include <franka/lowpass_filter.h>

#include "teleop_franka_joy/teleop_franka_joy.h"
#include "sensor_msgs/Joy.h"
#include "franka_msgs/FrankaState.h"

// Definir un namespace evita conflictos de nombres con otras partes del código o blibliotecas externas
namespace teleop_franka_joy
{

  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopFrankaJoy
   * directly into base nodes.
   */
  struct TeleopFrankaJoy::Impl
  {
    // Members functions
    void printTwistInfo(const geometry_msgs::Twist &velocity, const std::string &info_string);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy); // Función encargada de manejar los mensajes del joystick
    void sendCmdVel();

    // ROS subscribers and publisher
    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub;

    double Delta_t = 0.001; // Tiempo en segundo
    float reaction_t = 0.5; // Tiempo en segundo de reaccion del operador

    double alpha_first_order;

    std::array<double, 6> last_O_dP_EE_c;
    std::array<double, 6> last_O_ddP_EE_c;

    std::array<double, 6> O_dP_EE_c;
    std::array<double, 6> O_dP_EE_c_limited;

    int enable_mov_position;    // Variable que activa el control
    int enable_mov_orientation; // Variable que activa la velocidad orientation
    int home_button;

    // Creación de un map de ejes por cada tipo de control:
    std::map<std::string, int> axis_linear_map;  // Control de posicion
    std::map<std::string, int> axis_angular_map; // Control de orientación
  };

  /**
   * Constructs TeleopFrankaJoy.
   * \param nh NodeHandle to use for setting up the publisher and subscriber.
   * \param nh_param NodeHandle to use for searching for configuration parameters.
   */
  // Constructor: Inicializa los parámetros del nodo ROS y los parámetros del joystick
  TeleopFrankaJoy::TeleopFrankaJoy(ros::NodeHandle *nh, ros::NodeHandle *nh_param)
  {
    pimpl_ = new Impl;

    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_franka_vel", 1, true);                           // Se crea el publicador ROS
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopFrankaJoy::Impl::joyCallback, pimpl_); // Cuando se recibe un mensaje llama a la función callback.

    // Asignar botones
    nh_param->param<int>("enable_mov_position", pimpl_->enable_mov_position, 0); // Se obtiene el parámetro del enable_mov_position del servidor de parámetros ROS, por defecto es 0.
    nh_param->param<int>("enable_mov_orientation", pimpl_->enable_mov_orientation, -1);
    nh_param->param<int>("home_button", pimpl_->home_button, -1);

    // Asignación de mapas
    nh_param->getParam("axis_linear_map", pimpl_->axis_linear_map);
    nh_param->getParam("axis_angular_map", pimpl_->axis_angular_map);

    // Asignar valores a vbles
    pimpl_->O_dP_EE_c = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    pimpl_->last_O_dP_EE_c = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    pimpl_->last_O_ddP_EE_c = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  }

  void TeleopFrankaJoy::Impl::printTwistInfo(const geometry_msgs::Twist &velocity, const std::string &info_string)
  {
    ROS_INFO("%s - Linear (x, y, z): (%.5f, %.5f, %.5f), Angular (x, y, z, w): (%.5f, %.5f, %.5f)",
             info_string.c_str(),
             velocity.linear.x, velocity.linear.y, velocity.linear.z,
             velocity.angular.x, velocity.angular.y, velocity.angular.z);
  }

  // Obtiene valores específicos del mensaje del joystick
  double getVal(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_map, const std::string &fieldname)
  {
    /*
    Método que obtiene valores especificos del mensaje del joystick:
    Argumentos:
      - joy_msg: mensaje joy del cual se va a obtener la informacion
      - axis_map: mapa de ejes de control
      - fieldname: campo que se quiere obtener [x,y,z] o [x,y,z,w]
    */

    if (axis_map.find(fieldname) == axis_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname))
    {
      return 0.0;
      ROS_INFO("Fallo de función getVal");
    }
    ROS_INFO("Entro");
    return joy_msg->axes[axis_map.at(fieldname)];
  }

  double firstOrderFilter(double value, double previous_value, double alpha)
  {
    return alpha * value + (1 - alpha) * previous_value;
  }

  std::array<double, 6> firstOrderFilter(const std::array<double, 6> array, const std::array<double, 6> last_array, double alpha)
  {
    std::array<double, 6> array_filtered;

    for (int i = 0; i < 6; i++)
    {
      array_filtered[i] = alpha * array[i] + (1 - alpha) * last_array[i];

      // Redondea las soluciones a 0
      if (std::abs(array_filtered[i]) < (1e-6))
      {
        array_filtered[i] = 0.0;
      }
    }
    return array_filtered;
  }

  std::array<double, 6> lowpassFilter_array(double Delta_t, const std::array<double, 6> array, const std::array<double, 6> last_array, double cutoff_frecuency)
  {
    std::array<double, 6> array_filtered;

    for (int i = 0; i < 6; i++)
    {
      array_filtered[i] = franka::lowpassFilter(Delta_t, array[i], last_array[i], cutoff_frecuency);

      // Redondea las soluciones a 0
      if (std::abs(array_filtered[i]) < (1e-6))
      {
        array_filtered[i] = 0.0;
      }
    }
    return array_filtered;
  }

  std::array<double, 6> calculateAceleration(const std::array<double, 6> array, const std::array<double, 6> last_array, double Delta_t)
  {
    std::array<double, 6> array_aceleration;

    for (int i = 0; i < 6; i++)
    {
      array_aceleration[i] = (array[i] - last_array[i]) / Delta_t;
    }
    return array_aceleration;
  }

  geometry_msgs::Twist array6toTwist(const std::array<double, 6> array)
  {
    geometry_msgs::Twist twist;

    twist.linear.x = array[0];
    twist.linear.y = array[1];
    twist.linear.z = array[2];

    twist.angular.x = array[3];
    twist.angular.y = array[4];
    twist.angular.z = array[5];

    return twist;
  }

  void TeleopFrankaJoy::Impl::sendCmdVel()
  {
    // Aplico limitRate a la velocidad
    O_dP_EE_c_limited = franka::limitRate(franka::kMaxTranslationalVelocity * 0.1, // limitacion de velocidad
                                          franka::kMaxTranslationalAcceleration * 0.1,
                                          franka::kMaxTranslationalJerk * 0.1,
                                          franka::kMaxRotationalVelocity * 0.1,
                                          franka::kMaxRotationalAcceleration * 0.1,
                                          franka::kMaxRotationalJerk * 0.1,
                                          O_dP_EE_c,
                                          last_O_dP_EE_c,
                                          last_O_ddP_EE_c);

    // Aplica el filtro de primer orden a la velocidad
    std::array<double, 6> O_dP_EE_c_filtered = firstOrderFilter(O_dP_EE_c_limited, last_O_dP_EE_c, alpha_first_order);
    //double cutoff_frecuency = 30;
    // std::array<double, 6> O_dP_EE_c_filtered = lowpassFilter_array(Delta_t, O_dP_EE_c_limited, last_O_dP_EE_c, cutoff_frecuency);

    // Aplicar filtro de primer orden a la aceleración
    std::array<double, 6> O_ddP_EE_c = calculateAceleration(O_dP_EE_c_filtered, last_O_dP_EE_c, Delta_t);

    // Prepara siguiente ciclo
    last_O_ddP_EE_c = O_ddP_EE_c;
    last_O_dP_EE_c = O_dP_EE_c_filtered;

    // Convertir Array en Twist
    geometry_msgs::Twist velocity_to_command = array6toTwist(O_dP_EE_c_limited);
    printTwistInfo(velocity_to_command, "Velocidad publicada");
    cmd_vel_pub.publish(velocity_to_command);

    ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
  }

  void TeleopFrankaJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    /*
    {Si no se necesitan condiciones especiales para la deceleración}
    Cambiar if para que:
      - Al pulsar LB: O_dP_EE_c usa getval(axis_linear_map)
      - Al pulsar RB: O_dP_EE_c usa getval(axis_angular_map)
      - Al no pulsar nada: 0_dP_EE_c es 0.0

    Fuera del if ya se hace todo el código que calcula la velocity_to_command y la publica.

    {Si hicieran falta un alpha distinto en la aceleración y la deceleración}
      - [A] Variar el alpha global en cada uno de los casos del if
      - [B] Calculo de alpha dinamico
    */
    if (joy_msg->buttons[enable_mov_position]) // Boton derecho
    {
      // Velocidad lineal
      ROS_INFO("Boton LB pulsado");
      alpha_first_order = 1;
      O_dP_EE_c = {{getVal(joy_msg, axis_linear_map, "x"),
                    getVal(joy_msg, axis_linear_map, "y"),
                    getVal(joy_msg, axis_linear_map, "z"),
                    0.0,
                    0.0,
                    0.0}};
    }
    else if (joy_msg->buttons[enable_mov_orientation]) // Boton izquierdo
    {
      // Velocidad angular
      ROS_INFO("Boton RB pulsado");
      alpha_first_order = 1;
      O_dP_EE_c = {{0.0,
                    0.0,
                    0.0,
                    getVal(joy_msg, axis_angular_map, "x"),
                    getVal(joy_msg, axis_angular_map, "y"),
                    getVal(joy_msg, axis_angular_map, "z")}};
    }
    else
    { // Si no se toca LB o RB -> Decelera
      alpha_first_order = 1;
      O_dP_EE_c = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    }

    sendCmdVel();
  }

} // namespace teleop_franka_joy
