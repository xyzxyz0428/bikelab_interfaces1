#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bikelab_interfaces.msg import BikeSpeedData, PowerMeterData  # Import custom messages
from openant.easy.node import Node as ANTNode
from openant.devices import ANTPLUS_NETWORK_KEY
from openant.devices.bike_speed_cadence import BikeSpeed, BikeSpeedData as ANT_BikeSpeedData
from openant.devices.power_meter import PowerMeter, PowerData

WHEEL_CIRCUMFERENCE_M = 2.20114

class BikeSensorNode(Node):
    def __init__(self, bike_speed_id=18412, power_meter_id=64434):
        super().__init__('bike_sensor_node')

        # Set up ROS publishers
        self.speed_publisher = self.create_publisher(BikeSpeedData, 'bike_speed_data', 10)
        self.power_publisher = self.create_publisher(PowerMeterData, 'power_meter_data', 10)

        # Initialize ANT+ node and devices
        self.ant_node = ANTNode()
        self.ant_node.set_network_key(0x00, ANTPLUS_NETWORK_KEY)

        self.bike_speed_device = BikeSpeed(self.ant_node, device_id=bike_speed_id)
        self.power_meter_device = PowerMeter(self.ant_node, device_id=power_meter_id)

        # Assign callbacks for ANT+ devices
        self.bike_speed_device.on_found = self.on_bike_speed_found
        self.bike_speed_device.on_device_data = self.on_bike_speed_data
        self.power_meter_device.on_found = self.on_power_meter_found
        self.power_meter_device.on_device_data = self.on_power_meter_data

        # Start ANT+ node
        self.get_logger().info("Starting ANT+ devices, press Ctrl-C to finish")
        self.ant_node.start()

    def on_bike_speed_found(self):
        self.get_logger().info(f"Bike Speed Device {self.bike_speed_device} found and receiving")

    def on_power_meter_found(self):
        self.get_logger().info(f"Power Meter Device {self.power_meter_device} found and receiving")

    def on_bike_speed_data(self, page: int, page_name: str, data):
        if isinstance(data, ANT_BikeSpeedData):
            speed = data.calculate_speed(WHEEL_CIRCUMFERENCE_M)
            distance = data.calculate_distance(WHEEL_CIRCUMFERENCE_M)

            # Only publish if speed is not None
            if speed is not None:
                speed_msg = BikeSpeedData()
                speed_msg.speed = speed
                speed_msg.distance = distance 
                speed_msg.header = Header()
                speed_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

                # Log and publish message
                self.get_logger().info("Publishing bike speed data")
                self.speed_publisher.publish(speed_msg)
    def on_power_meter_data(self, page: int, page_name: str, data):
        if isinstance(data, PowerData):
            # Prepare and publish PowerMeterData message
            power_msg = PowerMeterData()
            # Only publish if speed is not None
            if power_msg.torque is not None:
                power_msg.angular_velocity = float(data.angular_velocity) 
                power_msg.average_power = float(data.average_power) 
                power_msg.cadence = float(data.cadence)
                power_msg.instantaneous_power = float(data.instantaneous_power) 
                power_msg.left_power = float(data.left_power) 
                power_msg.right_power = float(data.right_power) 
                power_msg.torque =  float(data.torque) 
                power_msg.header = Header()
                power_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

                # Log and publish message
                self.get_logger().info("Publishing power meter data")
                self.power_publisher.publish(power_msg)

    def destroy_node(self):
        self.bike_speed_device.close_channel()
        self.power_meter_device.close_channel()
        self.ant_node.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bike_sensor_node = BikeSensorNode()

    try:
        rclpy.spin(bike_sensor_node)
    except KeyboardInterrupt:
        bike_sensor_node.get_logger().info("Shutting down bike_sensor_node.")
    finally:
        bike_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
