#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string frame_id;
  double time_offset_in_seconds;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;


  ros::init(argc, argv, "mpu9250_serial_to_imu_node");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 1);
  ros::Publisher trigger_time_pub = nh.advertise<sensor_msgs::TimeReference>("trigger_time", 1);


  ros::Rate r(400); // 400 hz

  sensor_msgs::Imu imu;

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;

  sensor_msgs::TimeReference trigger_time_msg;

  std::string input;
  std::string read;

  uint32_t lastTriggerCounter = 0;

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if(ser.available())
        {
          read = ser.read(ser.available());
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
          input += read;
          while (input.length() >= 25) // while there might be a complete package in input
          {
            //parse for data packets
            data_packet_start = input.find("$\x03");
            if (data_packet_start != std::string::npos)
            {
              ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
              if ((input.length() >= data_packet_start + 25) && (input.compare(data_packet_start + 23, 2, "\n\r")))  //check if positions 26,27 exist, then test values
              {
                ROS_DEBUG("seems to be a real data package: long enough and found end characters");

                // get acelerometer values
                int16_t ax = (((0xff &(char)input[data_packet_start + 2]) << 8) | 0xff &(char)input[data_packet_start + 3]);

                int16_t ay = (((0xff &(char)input[data_packet_start + 4]) << 8) | 0xff &(char)input[data_packet_start + 5]);

                int16_t az = (((0xff &(char)input[data_packet_start + 6]) << 8) | 0xff &(char)input[data_packet_start + 7]);

                // calculate accelerations in m/s²
                double axf = ax / 1000.0;
                double ayf = ay / 1000.0;
                double azf = az / 1000.0;


                // get gyro values
                int16_t gx = (((0xff &(char)input[data_packet_start + 8]) << 8) | 0xff &(char)input[data_packet_start + 9]);

                int16_t gy = (((0xff &(char)input[data_packet_start + 10]) << 8) | 0xff &(char)input[data_packet_start + 11]);

                int16_t gz = (((0xff &(char)input[data_packet_start + 12]) << 8) | 0xff &(char)input[data_packet_start + 13]);

                double gxf = gx / 1000.0;
                double gyf = gy / 1000.0;
                double gzf = gz / 1000.0;


                uint8_t received_message_number = input[data_packet_start + 14];
                ROS_DEBUG("received message number: %i", received_message_number);

                if (received_message) // can only check for continuous numbers if already received at least one packet
                {
                  uint8_t message_distance = received_message_number - last_received_message_number;
                  if ( message_distance > 1 )
                  {
                    ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU9250 data packets from arduino.");
                  }
                }
                else
                {
                  received_message = true;
                }
                last_received_message_number = received_message_number;

                // calculate measurement time
                uint32_t ts =   ( ((0xff &(char)input[data_packet_start + 15]) << 24)
                                | ((0xff &(char)input[data_packet_start + 16]) << 16)
			                         	| ((0xff &(char)input[data_packet_start + 17]) << 8)
				                        | 0xff &(char)input[data_packet_start + 18]);

                // get trigger counter
                uint32_t triggerCounter =  ( ((0xff &(char)input[data_packet_start + 19]) << 24)
                                            | ((0xff &(char)input[data_packet_start + 20]) << 16)
				                                    | ((0xff &(char)input[data_packet_start + 21]) << 8)
				                                    | 0xff &(char)input[data_packet_start + 22]);

		           ros::Time measurement_time(ts / 1000, (ts % 1000) * 1000*1000);  // sec, nsec

                // publish imu message
                imu.header.stamp = measurement_time;
                imu.header.frame_id = frame_id;

                imu.orientation.x = 0;
                imu.orientation.y = 0;
                imu.orientation.z = 0;
                imu.orientation.w = 0;

                imu.angular_velocity.x = gxf;
                imu.angular_velocity.y = gyf;
                imu.angular_velocity.z = gzf;

                imu.linear_acceleration.x = axf;
                imu.linear_acceleration.y = ayf;
                imu.linear_acceleration.z = azf;

                imu_pub.publish(imu);

	              // publish triggertime message
		            if (triggerCounter != lastTriggerCounter){
		              ros::Time time_ref(0, 0);
 		              trigger_time_msg.header.frame_id = frame_id;
                  trigger_time_msg.header.stamp = measurement_time;
                  trigger_time_msg.time_ref = time_ref;
                  trigger_time_pub.publish(trigger_time_msg);
  	   	          lastTriggerCounter = triggerCounter;
	             	}

                input.erase(0, data_packet_start + 25); // delete everything up to and including the processed packet
              }
              else
              {
                if (input.length() >= data_packet_start + 25)
                {
                  input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                }
                else
                {
                  // do not delete start character, maybe complete package has not arrived yet
                  input.erase(0, data_packet_start);
                }
              }
            }
            else
            {
              // no start character found in input, so delete everything
              input.clear();
            }
          }
        }
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(230400);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}
