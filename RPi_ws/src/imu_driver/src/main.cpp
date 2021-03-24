#include <stdio.h>
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"

void sig_handler(int s);
static bool got_ctrl_c = false;

///////////////////////////////////////////

// For efficiency in compiling, this is the only file that uses the
// Eigen lirbary and the 'vector' type we made from it.

#include "vector.h"
#include "version.h"
#include "prog_options.h"
#include "minimu9.h"
#include "exceptions.h"
#include "pacer.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <system_error>
#include <chrono>

// TODO: print warning if accelerometer magnitude is not close to 1 when starting up

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
float field_width = 6;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector)
{
  return os << FLOAT_FORMAT << vector(0) << ' '
            << FLOAT_FORMAT << vector(1) << ' '
            << FLOAT_FORMAT << vector(2);
}

std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
  return os << (vector)matrix.row(0) << ' '
            << (vector)matrix.row(1) << ' '
            << (vector)matrix.row(2);
}

std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
  return os << FLOAT_FORMAT << quat.w() << ' '
            << FLOAT_FORMAT << quat.x() << ' '
            << FLOAT_FORMAT << quat.y() << ' '
            << FLOAT_FORMAT << quat.z();
}

typedef void rotation_output_function(quaternion & rotation);

void output_quaternion(quaternion & rotation)
{
  std::cout << rotation;
}

void output_matrix(quaternion & rotation)
{
  std::cout << rotation.toRotationMatrix();
}

void output_euler(quaternion & rotation)
{
  std::cout << (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0)
                        * (180 / M_PI));
}

void stream_raw_values(imu & imu)
{
  imu.enable();
  while(1)
  {
    imu.read_raw();
    printf("%7d %7d %7d  %7d %7d %7d  %7d %7d %7d\n",
           imu.m[0], imu.m[1], imu.m[2],
           imu.a[0], imu.a[1], imu.a[2],
           imu.g[0], imu.g[1], imu.g[2]
      );
    usleep(20*1000);
  }
}

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix rotation_from_compass(const vector & acceleration, const vector & magnetic_field)
{
  vector down = -acceleration;     // usually true
  vector east = down.cross(magnetic_field); // actually it's magnetic east
  vector north = east.cross(down);

  east.normalize();
  north.normalize();
  down.normalize();

  matrix r;
  r.row(0) = north;
  r.row(1) = east;
  r.row(2) = down;
  return r;
}


typedef void fuse_function(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field);

void fuse_compass_only(quaternion & rotation, float dt, const vector& angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  // Implicit conversion of rotation matrix to quaternion.
  rotation = rotation_from_compass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion & rotation, const vector & w, float dt)
{
  // Multiply by first order approximation of the
  // quaternion representing this rotation.
  rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
  rotation.normalize();
}

void fuse_gyro_only(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  rotate(rotation, angular_velocity, dt);
}

void fuse_default(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  vector correction = vector(0, 0, 0);

  if (fabs(acceleration.norm() - 1) <= 0.3)
  {
    // The magnetidude of acceleration is close to 1 g, so
    // it might be pointing up and we can do drift correction.

    const float correction_strength = 1;

    matrix rotation_compass = rotation_from_compass(acceleration, magnetic_field);
    matrix rotation_matrix = rotation.toRotationMatrix();

    correction = (
      rotation_compass.row(0).cross(rotation_matrix.row(0)) +
      rotation_compass.row(1).cross(rotation_matrix.row(1)) +
      rotation_compass.row(2).cross(rotation_matrix.row(2))
      ) * correction_strength;

  }

  rotate(rotation, angular_velocity + correction, dt);
}


// send tf as well as the orientation
void ahrs_ros(imu & imu, fuse_function * fuse, rotation_output_function * output)
{
  ros::NodeHandle node;
  static ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("imu", 1
  );
  sensor_msgs::Imu imu_msg;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;

  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();

  // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();

  ros::NodeHandle private_node_handle_("~");
  ros::Rate loop_rate(50); // this is what the author of the imu code used

  auto start = std::chrono::steady_clock::now();
  while (ros::ok()) 
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

    // Print the euler angles to the terminal
    //output(rotation);
    //std::cout << "  " << acceleration << "  " << magnetic_field << std::endl;
    //std::cout << "  " << acceleration << "  " << angular_velocity << std::endl;

    // Publishes the IMU message
    imu_msg.orientation.x = rotation.x();
    imu_msg.orientation.y = rotation.y();
    imu_msg.orientation.z = rotation.z();
    imu_msg.orientation.w = rotation.w();

    imu_msg.linear_acceleration.x = 9.8*acceleration.x();
    imu_msg.linear_acceleration.y = 9.8*acceleration.y();
    imu_msg.linear_acceleration.z = 9.8*acceleration.z();

    imu_msg.angular_velocity.x = angular_velocity.x();
    imu_msg.angular_velocity.y = angular_velocity.y();
    imu_msg.angular_velocity.z = angular_velocity.z();

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_imu";
    imu_pub.publish(imu_msg);

    /*
    // send the transform
    odom_trans.header.stamp =  ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "odom";
    odom_trans.transform.rotation.x = rotation.x();
    odom_trans.transform.rotation.y = rotation.y();
    odom_trans.transform.rotation.z = rotation.z();
    odom_trans.transform.rotation.w = rotation.w();
    odom_broadcaster.sendTransform(odom_trans);
    */

    if (got_ctrl_c) {
      ROS_WARN("Got Ctrl-C");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}


void ahrs(imu & imu, fuse_function * fuse, rotation_output_function * output)
{
  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();

  // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(20000000);

  auto start = std::chrono::steady_clock::now();
  while(1)
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

    output(rotation);
    std::cout << "  " << acceleration << "  " << magnetic_field << std::endl;

    loop_pacer.pace();
  }
}

int main_with_exceptions(int argc, char **argv)
{
  prog_options options = get_prog_options(argc, argv);

  if(options.show_help)
  {
    print_command_line_options_desc();
    std::cout << "For more information, run: man minimu9-ahrs" << std::endl;
    return 0;
  }

  if (options.show_version)
  {
    std::cout << VERSION << std::endl;
    return 0;
  }
  
  // Decide what sensors we want to use.
  sensor_set set;
  set.mag = set.acc = set.gyro = true;

  minimu9::comm_config config = minimu9::auto_detect("/dev/i2c-1");

  sensor_set missing = set - minimu9::config_sensor_set(config);
  if (missing)
  {
    if (missing.mag)
    {
      std::cerr << "Error: No magnetometer found." << std::endl;
    }
    if (missing.acc)
    {
      std::cerr << "Error: No accelerometer found." << std::endl;
    }
    if (missing.gyro)
    {
      std::cerr << "Error: No gyro found." << std::endl;
    }
    std::cerr << "Error: Needed sensors are missing." << std::endl;
    return 1;
  }

  config = minimu9::disable_redundant_sensors(config, set);

  minimu9::handle imu;
  imu.open(config);

  rotation_output_function * output;

  field_width += 2;  // See comment above for field_width.
  output = &output_euler; // Output euler angles in terminal
  
  // fuse_default: Fuse compass and gyro
  ahrs_ros(imu, &fuse_default, output);
  //ahrs_ros(imu, &fuse_compass_only, output);
  //ahrs_ros(imu, &fuse_gyro_only, output);

  return 0;
}

///////////////////////////////////////////

int main(int argc, char *argv[])
{
  printf("Hello from minimu9-ahrs!\n");

  signal(SIGINT, sig_handler);

  ros::init(argc, argv, "minimu9");

  main_with_exceptions(argc, argv);

  ROS_WARN("Exiting.");

  exit(0);
}

void sig_handler(int s)
{
  if (s == 2) {
    got_ctrl_c = true; 
  }
}

