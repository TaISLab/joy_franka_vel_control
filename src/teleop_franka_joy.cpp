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
    double smooth_increment(double &incremento, const double &limite_inferior, const double &limite_superior);

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);                                                                // Función encargada de manejar los mensajes del joystick
    void sendCmdLinearVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_linear_map); // Función encargada de calcular los valores de PoseStamped
    void sendCmdAngularVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_angular_map);
    void ModifyVelocity(const sensor_msgs::Joy::ConstPtr &joy_msg, float &scale, float &max_vel); // Función encargada de modificar la velocidad
    void obtainEquilibriumPose(const franka_msgs::FrankaStateConstPtr &msg);

    // ROS subscribers and publisher
    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub;

    float Delta_t = 0.001;  // Tiempo en segundo
    float reaction_t = 0.5; // Tiempo en segundo de reaccion del operador

    double last_commanded_velocity;
    double last_commanded_acceleration;

    std::array<double, 6> last_O_dP_EE_c;
    std::array<double, 6> last_O_ddP_EE_c;
    double kFactor = 1;

    std::array<double, 6> velocity;

    ros::Time elapsed_time_;

    int enable_mov_position;    // Variable que activa el control
    int enable_mov_orientation; // Variable que activa la velocidad orientation
    int orientation_button;
    int home_button;
    int increment_vel;
    int decrement_vel;

    float linear_max_vel;
    float angular_max_vel; // max_displacement_in_a_second
    float min_vel = 2;

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

    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);                           // Se crea el publicador ROS
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopFrankaJoy::Impl::joyCallback, pimpl_); // Cuando se recibe un mensaje llama a la función callback.

    // Asignar botones
    nh_param->param<int>("enable_mov_position", pimpl_->enable_mov_position, 0); // Se obtiene el parámetro del enable_mov_position del servidor de parámetros ROS, por defecto es 0.
    nh_param->param<int>("enable_mov_orientation", pimpl_->enable_mov_orientation, -1);
    nh_param->param<int>("orientation_button", pimpl_->orientation_button, -1); // Antes 8
    nh_param->param<int>("home_button", pimpl_->home_button, -1);

    // Asignación de mapas
    nh_param->getParam("axis_linear_map", pimpl_->axis_linear_map);
    nh_param->getParam("axis_angular_map", pimpl_->axis_angular_map);

    nh_param->getParam("linear_max_vel", pimpl_->linear_max_vel);
    nh_param->getParam("angular_max_vel", pimpl_->angular_max_vel);

    // Asignar valores a vbles
    pimpl_->last_commanded_velocity = 0;
    pimpl_->last_commanded_acceleration = 0;
  }

  void TeleopFrankaJoy::Impl::printTwistInfo(const geometry_msgs::Twist &velocity, const std::string &info_string)
  {
    ROS_INFO("%s - Linear (x, y, z): (%.2f, %.2f, %.2f), Angular (x, y, z, w): (%.2f, %.2f, %.2f)",
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

  double applyLimits(double value, double min_limit, double max_limit)
  {
    // Aplica limites a la velocidad
    return std::min(std::max(value, min_limit), max_limit);
  }

  double TeleopFrankaJoy::Impl::smooth_increment(double &incremento, const double &limite_inferior, const double &limite_superior)
  {
    ROS_INFO("Entro a smooth increment");
    if (incremento != 0.0)
    {
      if (std::abs(incremento) > limite_superior)
      {
        // Aceleración
        ROS_INFO("Entro a limite superior");
        return std::copysign(limite_superior, incremento);
      }
      else if (std::abs(incremento) < limite_inferior)
      {
        // Deceleracion
        ROS_INFO("Entro a limite inferior");
        return std::copysign(limite_inferior, incremento);
      }
      else
      {
        // Fuera de limites
        ROS_INFO("Asigno incremento %.2f", incremento);
        return incremento;
      }
    }
    return 0.0;
  }

 
 double firstOrderFilter(double value, double previous_value, double alpha){

    return alpha * value + (1-alpha) * previous_value;

  }

  void TeleopFrankaJoy::Impl::sendCmdLinearVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_linear_map)
  {
    /*
      https://robotics.stackexchange.com/questions/18237/what-is-a-velocity-smoother
      https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article

      Velocity smoother:

      if target_speed > control_speed:
        control_speed = min(target_speed, control_speed*0.02)

      else if target_speed < control_speed:
        control_speed = max(target_speed, control_speed - 0.02)
      else:
        control_speed = target_speed;

    */
    // geometry_msgs::Twist incremento_vel;
    // // Calcular incrementos
    // incremento_vel.linear.x = Delta_t * getVal(joy_msg, axis_linear_map, "x");
    // ROS_INFO("Incremento linear.x: %.2f", getVal(joy_msg, axis_linear_map, "x"));
    // // velocity.linear.x =  smooth_increment(incremento_vel.linear.x, 0.005, 0.01);

    // if (incremento_vel.linear.x > 0.01){
    //   incremento_vel.linear.x = 0.001;
    // }

    // velocity.linear.x += incremento_vel.linear.x;
    // // printTwistInfo(velocity, "Desired Velocity");
    // cmd_vel_pub.publish(velocity);

    // ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
    // ROS_INFO("Espera de Delta_t completada.");

    // double commanded_velocity = getVal(joy_msg, axis_linear_map, "x");
    // double commanded_velocity = 1;

    const std::array<double, 6> O_dP_EE_c = {{getVal(joy_msg, axis_linear_map, "x")*kFactor, 0.0, 0.0, 0.0, 0.0, 0.0}};

    

    velocity = franka::limitRate(franka::kMaxTranslationalVelocity*0.01,
                                 franka::kMaxTranslationalAcceleration,
                                 franka::kMaxTranslationalJerk * 0.05 * kFactor,
                                 franka::kMaxRotationalVelocity,
                                 franka::kMaxRotationalAcceleration,
                                 franka::kMaxRotationalJerk,
                                 O_dP_EE_c,
                                 last_O_dP_EE_c,
                                 last_O_ddP_EE_c);

    double vel_vx_filter = firstOrderFilter(velocity[0], last_O_dP_EE_c[0], 0.2);

    last_O_ddP_EE_c = {{(vel_vx_filter - last_O_dP_EE_c[0]) / Delta_t, 0.0, 0.0, 0.0, 0.0, 0.0}};
    last_O_dP_EE_c = {{velocity[0], 0.0, 0.0, 0.0, 0.0, 0.0}};

    ROS_INFO("Vx_limitRate=%.6f", velocity[0]);
    geometry_msgs::Twist velocity_to_command;
    velocity_to_command.linear.x = velocity[0];
    cmd_vel_pub.publish(velocity_to_command);
    ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
  }

  void TeleopFrankaJoy::Impl::sendCmdAngularVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_angular_map)
  {
    // // ROS usa dos tipos de quaternions que no se peuden mezclar pero si convertir
    // tf2::Quaternion q_rot;
    // tf2::Quaternion q_new;
    // geometry_msgs::Twist velocity;

    // // Calculo de quaternion en funcion de angulos eulerXYZ
    // double roll = getVal(joy_msg, axis_angular_map, "x");
    // double pitch = getVal(joy_msg, axis_angular_map, "y");
    // double yaw = getVal(joy_msg, axis_angular_map, "z");
    // q_rot.setRPY(roll, pitch, yaw);

    // // Aplicar rotacion
    // q_rot;

    // // Normalizar
    // q_rot.normalize();

    // // convertir de tf2 a msg
    // tf2::convert(q_new, velocity.angular);

    // // Publicar nueva orientacion
    // cmd_vel_pub.publish(velocity);
    // printTwistInfo(velocity, "Desired Velocity");

    // // ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
    // // ROS_INFO("Espera de Delta_t completada.");
  }

  void TeleopFrankaJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {

    if (joy_msg->buttons[enable_mov_position]) // Boton derecho
    {
      ROS_INFO("Boton LB pulsado");
      sendCmdLinearVelMsg(joy_msg, axis_linear_map);
    }
    else if (joy_msg->buttons[enable_mov_orientation]) // Boton izquierdo
    {
      ROS_INFO("Boton RB pulsado");

      sendCmdAngularVelMsg(joy_msg, axis_angular_map);
    }
    else
    { // Si no se toca nada

      // // Función deceleradora
      // geometry_msgs::Twist velocity_increment;
      // velocity_increment.linear.x = 0.0;
      // velocity.linear.x = velocity.linear.x + smooth_increment(velocity_zero.linear.x, 0.0005, 0.01);
      // cmd_vel_pub.publish(velocity); // Se publica el equilibrium_pose cuando no se pulsa ninguna tecla
      // printTwistInfo(velocity, "Velocity set to 0");

      

      const std::array<double, 6> O_dP_EE_c = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      velocity = franka::limitRate(franka::kMaxTranslationalVelocity * 0.015,
                                   franka::kMaxTranslationalAcceleration,
                                   franka::kMaxTranslationalJerk * 0.05 * kFactor,
                                   franka::kMaxRotationalVelocity,
                                   franka::kMaxRotationalAcceleration,
                                   franka::kMaxRotationalJerk,
                                   O_dP_EE_c,
                                   last_O_dP_EE_c,
                                   last_O_ddP_EE_c);

      last_O_ddP_EE_c = {{(velocity[0] - last_O_dP_EE_c[0]) / Delta_t, 0.0, 0.0, 0.0, 0.0, 0.0}};
      last_O_dP_EE_c = {{velocity[0], 0.0, 0.0, 0.0, 0.0, 0.0}};

      ROS_INFO("Vx_limitRate=%.6f", velocity[0]);
      geometry_msgs::Twist velocity_to_command;
      velocity_to_command.linear.x = velocity[0];
      cmd_vel_pub.publish(velocity_to_command);
      ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
    }
  }

} // namespace teleop_franka_joy