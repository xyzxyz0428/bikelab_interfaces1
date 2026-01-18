#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bikelab_interfaces.msg import AdcData
import smbus
from std_msgs.msg import Header
import time

##config
ADC_minus_90=188
ADC_minus_45= 363
ADC_0= 514
ADC_45= 688
ADC_90= 851

class PotentiometerDriver(Node):
    def __init__(self):
        super().__init__('potentiometer_driver')
        self.publisher_ = self.create_publisher(AdcData, 'adc_data', 10)
        self.ADC = smbus.SMBus(1)  # Setup I2C communication on bus 1
        self.timer = self.create_timer(0.1, self.timer_callback)  # Create a timer to call the callback every 1 second

    def timer_callback(self):
        try:
            self.ADC.write_byte(0x24, 0x10)  # Write a byte to the slave
            data_value = self.ADC.read_word_data(0x24, 0x10)  # Read data from the ADC
            self.get_logger().info("Data read from ADC: {}".format(data_value))

            # Create a message and assign the data and timestamp
            msg = AdcData()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
            # Define calibration points: (adc_value, angle)
            calibration_points = [
                (ADC_minus_90, -90),
                (ADC_minus_45,-45),
                (ADC_0, 0),
                (ADC_45, 45),
                (ADC_90, 90)
            ]

            # Linear interpolation between calibration points
            def interpolate_angle(adc_value, points):
                for i in range(len(points) - 1):
                    x0, y0 = points[i]
                    x1, y1 = points[i + 1]
                    if x0 <= adc_value <= x1:
                        # Linear interpolation formula
                        return y0 + (y1 - y0) * (adc_value - x0) / (x1 - x0)
                # If out of range, clamp to min/max
                if adc_value < points[0][0]:
                    return points[0][1]
                if adc_value > points[-1][0]:
                    return points[-1][1]
                return 0.0  # fallback

            msg.data = float(interpolate_angle(data_value, calibration_points))
            #msg.adc_data = float(data_value)
            self.get_logger().info("steering angle: {}".format(msg.data))
            self.publisher_.publish(msg)  # Publish the message
        except Exception as e:
            self.get_logger().error("Error reading ADC data: {}".format(e))

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication
    potentiometer_driver = PotentiometerDriver()  # Create the ROS node
    rclpy.spin(potentiometer_driver)  # Keep the node alive to listen to callbacks
    potentiometer_driver.destroy_node()  # Cleanup the node
    rclpy.shutdown()  # Shutdown ROS communication

if __name__ == '__main__':
    main()
