import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, BatteryState
from mavros_msgs.msg import RCOut
from builtin_interfaces.msg import Time
from pymavlink import mavutil
import threading

class MAVLinkBridge(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')
        self.publisher = self.create_publisher(FluidPressure, 'pressure/bar30', 10)
        self.pub_rcout = self.create_publisher(RCOut, '/mavros/rc/out', 10)
        self.pub_battery = self.create_publisher(BatteryState, 'battery/status', 10)

        self.mavlink_connection = mavutil.mavlink_connection('udpin:192.168.2.103:15550')
        self.get_logger().info("Waiting for MAVLink messages...")
        self.mavlink_connection.wait_heartbeat()

        # Start a separate thread for receiving MAVLink messages
        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()

    def recv_loop(self):
        while rclpy.ok():
            mav_msg = self.mavlink_connection.recv_match(blocking=True)
            if mav_msg is None:
                continue

            msg_type = mav_msg.get_type()

            if msg_type == 'SCALED_PRESSURE2':
                pressure_msg = FluidPressure()
                pressure_msg.header.stamp = self.get_clock().now().to_msg()
                pressure_msg.fluid_pressure = mav_msg.press_abs * 100
                self.publisher.publish(pressure_msg)

            elif msg_type == 'SERVO_OUTPUT_RAW':
                rc_msg = RCOut()
                ts = mav_msg._timestamp
                sec = int(ts)
                nanosec = int((ts - sec) * 1e9)
                
                rc_msg.header.stamp = Time(sec=sec, nanosec=nanosec)

                rc_msg.channels = [
                    mav_msg.servo1_raw,
                    mav_msg.servo2_raw,
                    mav_msg.servo3_raw,
                    mav_msg.servo4_raw,
                    mav_msg.servo5_raw,
                    mav_msg.servo6_raw,
                    mav_msg.servo7_raw,
                    mav_msg.servo8_raw,
                    mav_msg.servo9_raw,
                    mav_msg.servo10_raw,
                    mav_msg.servo11_raw,
                    mav_msg.servo12_raw,
                    mav_msg.servo13_raw,
                    mav_msg.servo14_raw,
                    mav_msg.servo15_raw,
                    mav_msg.servo16_raw,
                ]
                self.pub_rcout.publish(rc_msg)

            elif msg_type == 'BATTERY_STATUS':
                msg = BatteryState()
                # -----------------------------
                # Timestamp
                # -----------------------------
                ts = mav_msg._timestamp  # seconds (float)
                sec = int(ts)
                nanosec = int((ts - sec) * 1e9)
                msg.header.stamp = Time(sec=sec, nanosec=nanosec)
                msg.header.frame_id = "battery"

                # -----------------------------
                # Voltage (average of valid cells)
                # MAVLink: mV, ROS: V
                # -----------------------------
                valid_cells = [
                    v for v in mav_msg.voltages
                    if v != 65535 and v > 0
                ]

                if valid_cells:
                    msg.voltage = sum(valid_cells) / len(valid_cells) / 1000.0
                else:
                    msg.voltage = float('nan')

                # -----------------------------
                # Current
                # MAVLink: centi-amps, ROS: amps
                # -----------------------------
                if mav_msg.current_battery != -1:
                    msg.current = mav_msg.current_battery / 100.0
                else:
                    msg.current = float('nan')

                # -----------------------------
                # Percentage
                # MAVLink: percent, ROS: [0.0, 1.0]
                # -----------------------------
                if mav_msg.battery_remaining >= 0:
                    msg.percentage = mav_msg.battery_remaining / 100.0
                else:
                    msg.percentage = float('nan')

                # -----------------------------
                # Temperature
                # MAVLink: centi-degC, ROS: degC
                # -----------------------------
                if mav_msg.temperature != 32767:
                    msg.temperature = mav_msg.temperature / 100.0
                else:
                    msg.temperature = float('nan')

                # -----------------------------
                # Charge (best-effort)
                # MAVLink: mAh, ROS: Ah
                # -----------------------------
                if mav_msg.current_consumed >= 0:
                    msg.charge = mav_msg.current_consumed / 1000.0
                else:
                    msg.charge = float('nan')


                # -----------------------------
                # Status
                # -----------------------------
                msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
                msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
                msg.present = True

                self.pub_battery.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
