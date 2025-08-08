#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
import time
import math
import threading

# Try multiple GPIO libraries for maximum compatibility
GPIO_AVAILABLE = False
GPIO_LIB = None

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    GPIO_LIB = "RPi.GPIO"
except ImportError:
    try:
        import Jetson.GPIO as GPIO
        GPIO_AVAILABLE = True
        GPIO_LIB = "Jetson.GPIO"
    except ImportError:
        try:
            import gpiod
            GPIO_AVAILABLE = True
            GPIO_LIB = "gpiod"
        except ImportError:
            GPIO_AVAILABLE = False

class HCSR04_Driver(Node):
    """
    HC-SR04 Ultrasonic Sensor Driver for ROS 2
    Optimized for Jetson Xavier deployment with production-ready features
    """

    def __init__(self):
        super().__init__("hcsr04_driver")
        
        # Sensor configuration parameters
        self.declare_parameter('trig_pin', 18)
        self.declare_parameter('echo_pin', 24)
        self.declare_parameter('frequency', 20.0)  # Hz
        self.declare_parameter('field_of_view', 0.5)  # radians (~30 degrees)
        self.declare_parameter('min_range', 0.02)  # meters
        self.declare_parameter('max_range', 4.0)   # meters
        self.declare_parameter('timeout_ms', 30.0)  # milliseconds
        self.declare_parameter('use_median_filter', True)
        self.declare_parameter('filter_window_size', 5)
        
        # Get parameters
        self.trig_pin = self.get_parameter('trig_pin').value
        self.echo_pin = self.get_parameter('echo_pin').value
        self.frequency = self.get_parameter('frequency').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.timeout_s = self.get_parameter('timeout_ms').value / 1000.0
        self.use_filter = self.get_parameter('use_median_filter').value
        self.filter_size = self.get_parameter('filter_window_size').value
        
        # Measurement filtering
        self.measurement_buffer = []
        self.lock = threading.Lock()
        
        # GPIO Setup
        self.is_connected = False
        self.gpio_chip = None
        self.init_gpio()
        
        # ROS 2 Interface
        self.range_pub = self.create_publisher(
            Range, 
            "/ultrasonic_range", 
            qos_profile=qos_profile_sensor_data
        )
        
        # Range message template
        self.range_msg = Range()
        self.range_msg.header.frame_id = "ultrasonic_link"
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = self.field_of_view
        self.range_msg.min_range = self.min_range
        self.range_msg.max_range = self.max_range
        
        # Timer for periodic measurements
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Simulation mode fallback values
        self.sim_range = 1.5  # Default simulation range
        self.sim_counter = 0
        
        # Status reporting
        self.measurement_count = 0
        self.error_count = 0
        
        if GPIO_AVAILABLE:
            self.get_logger().info(f"HC-SR04 driver initialized using {GPIO_LIB} with pins TRIG={self.trig_pin}, ECHO={self.echo_pin}")
        else:
            self.get_logger().warn("No GPIO library available. Running in simulation mode - perfect for development!")
            self.get_logger().info("To enable hardware mode on Jetson Xavier, install: sudo apt install python3-jetson-gpio")

    def init_gpio(self):
        """Initialize GPIO pins for HC-SR04 sensor with multi-platform support"""
        if not GPIO_AVAILABLE:
            self.is_connected = False
            return
            
        try:
            if GPIO_LIB == "gpiod":
                # Modern GPIO approach for newer systems
                self.gpio_chip = gpiod.Chip('gpiochip0')
                self.trig_line = self.gpio_chip.get_line(self.trig_pin)
                self.echo_line = self.gpio_chip.get_line(self.echo_pin)
                self.trig_line.request(consumer="hcsr04_trig", type=gpiod.LINE_REQ_DIR_OUT)
                self.echo_line.request(consumer="hcsr04_echo", type=gpiod.LINE_REQ_DIR_IN)
                self.trig_line.set_value(0)
                self.is_connected = True
                self.get_logger().info("GPIO initialized using modern gpiod library")
            else:
                # Traditional GPIO approach (RPi.GPIO or Jetson.GPIO)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.trig_pin, GPIO.OUT)
                GPIO.setup(self.echo_pin, GPIO.IN)
                GPIO.output(self.trig_pin, False)
                time.sleep(0.1)  # Allow sensor to settle
                self.is_connected = True
                self.get_logger().info(f"GPIO initialized using {GPIO_LIB}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            self.get_logger().error("Falling back to simulation mode")
            self.is_connected = False

    def median_filter(self, new_value):
        """Apply median filter to reduce noise"""
        with self.lock:
            self.measurement_buffer.append(new_value)
            if len(self.measurement_buffer) > self.filter_size:
                self.measurement_buffer.pop(0)
            
            if len(self.measurement_buffer) >= 3:
                sorted_buffer = sorted(self.measurement_buffer)
                return sorted_buffer[len(sorted_buffer) // 2]
            else:
                return new_value

    def measure_distance(self):
        """Measure distance using HC-SR04 ultrasonic sensor with improved reliability"""
        if not self.is_connected:
            # Enhanced simulation mode with realistic behavior
            self.sim_counter += 1
            # Simulate realistic environment with obstacles and noise
            base_distance = 1.2 + 0.8 * math.sin(self.sim_counter * 0.05)  # Slower variation
            # Add realistic measurement noise
            noise = 0.02 * (2 * (hash(self.sim_counter) % 100) / 100.0 - 1)
            simulated_distance = max(self.min_range, min(self.max_range, base_distance + noise))
            
            # Simulate occasional obstacles
            if self.sim_counter % 100 == 0:  # Every 5 seconds at 20Hz
                simulated_distance = 0.3  # Close obstacle
            
            return simulated_distance
        
        try:
            if GPIO_LIB == "gpiod":
                # Modern GPIO measurement
                self.trig_line.set_value(1)
                time.sleep(0.00001)  # 10 microseconds pulse
                self.trig_line.set_value(0)
                
                # Wait for echo to start
                timeout = time.time() + self.timeout_s
                while self.echo_line.get_value() == 0:
                    pulse_start = time.time()
                    if pulse_start > timeout:
                        self.error_count += 1
                        return float('inf')  # Timeout
                
                # Wait for echo to end
                timeout = time.time() + self.timeout_s
                while self.echo_line.get_value() == 1:
                    pulse_end = time.time()
                    if pulse_end > timeout:
                        self.error_count += 1
                        return float('inf')  # Timeout
            else:
                # Traditional GPIO measurement
                GPIO.output(self.trig_pin, True)
                time.sleep(0.00001)  # 10 microseconds pulse
                GPIO.output(self.trig_pin, False)
                
                # Wait for echo to start
                timeout = time.time() + self.timeout_s
                while GPIO.input(self.echo_pin) == 0:
                    pulse_start = time.time()
                    if pulse_start > timeout:
                        self.error_count += 1
                        return float('inf')  # Timeout
                
                # Wait for echo to end
                timeout = time.time() + self.timeout_s
                while GPIO.input(self.echo_pin) == 1:
                    pulse_end = time.time()
                    if pulse_end > timeout:
                        self.error_count += 1
                        return float('inf')  # Timeout
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound = 343 m/s / 2
            
            # Validate measurement
            if distance < self.min_range or distance > self.max_range:
                self.error_count += 1
                return float('inf')  # Out of range
                
            self.measurement_count += 1
            return distance
            
        except Exception as e:
            self.get_logger().error(f"Error measuring distance: {e}")
            self.error_count += 1
            return float('inf')

    def timer_callback(self):
        """Timer callback to publish range measurements with advanced features"""
        if not self.is_connected and GPIO_AVAILABLE:
            # Attempt reconnection if hardware becomes available
            self.init_gpio()
        
        # Measure distance
        raw_distance = self.measure_distance()
        
        # Apply filtering if enabled
        if self.use_filter and raw_distance != float('inf'):
            filtered_distance = self.median_filter(raw_distance)
        else:
            filtered_distance = raw_distance
        
        # Populate range message
        self.range_msg.header.stamp = self.get_clock().now().to_msg()
        self.range_msg.range = filtered_distance
        
        # Publish measurement
        self.range_pub.publish(self.range_msg)
        
        # Periodic status reporting
        if self.measurement_count > 0 and self.measurement_count % 200 == 0:  # Every 10 seconds at 20Hz
            error_rate = (self.error_count / self.measurement_count) * 100
            mode = "hardware" if self.is_connected else "simulation"
            self.get_logger().info(
                f"Ultrasonic sensor status: {mode} mode, "
                f"measurements: {self.measurement_count}, "
                f"error rate: {error_rate:.1f}%, "
                f"current range: {filtered_distance:.3f}m"
            )

    def destroy_node(self):
        """Clean up GPIO resources and provide final statistics"""
        if GPIO_AVAILABLE and self.is_connected:
            try:
                if GPIO_LIB == "gpiod" and self.gpio_chip:
                    self.gpio_chip.close()
                elif GPIO_LIB in ["RPi.GPIO", "Jetson.GPIO"]:
                    GPIO.cleanup()
                self.get_logger().info("GPIO resources cleaned up successfully")
            except Exception as e:
                self.get_logger().error(f"Error during GPIO cleanup: {e}")
        
        # Final statistics
        if self.measurement_count > 0:
            error_rate = (self.error_count / self.measurement_count) * 100
            self.get_logger().info(f"Final statistics: {self.measurement_count} measurements, {error_rate:.1f}% error rate")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        hcsr04_driver = HCSR04_Driver()
        rclpy.spin(hcsr04_driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'hcsr04_driver' in locals():
            hcsr04_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
