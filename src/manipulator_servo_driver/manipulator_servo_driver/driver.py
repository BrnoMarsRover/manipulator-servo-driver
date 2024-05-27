import rclpy
import math

import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mks_servo import MksServo
from mks_enums import Direction
import can

from manipulator_servo_driver_interfaces.srv import ChangeMode, ResetAxis


class ServoDriver(Node):
    def __init__(self):
        super().__init__("mks_servo_driver")
        self.JOINT_CNT = 6

        self.subscriber = self.create_subscription(JointState, "mks_servo_joints_cmd", self.on_command, 10)
        self.publisher = self.create_publisher(JointState, "mks_servo_joints", 10)
        self.service_change_mode = self.create_service(ChangeMode, "mks_servo_change_mode", self.on_change_mode)
        self.service_reset_axis = self.create_service(ResetAxis, "mks_servo_reset_axis", self.on_reset_axis)

        self.timer = self.create_timer(0.05, self.update_state)
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=125000)
        self.notifier = can.Notifier(self.bus, [])
        self.servos = [MksServo(self.bus, self.notifier, i) for i in range(1, self.JOINT_CNT + 1)]
        #self.servos = [MksServo(self.bus, self.notifier, 3)]
        self.mode = 1  # mode: 0 - standby, 1 - position, 2 - speed (currently quite dangerous with the hardware)

        for servo in self.servos:
            self.servo_init(servo)

    # MISC

    def servo_init(self, servo):
        servo.set_work_mode(MksServo.WorkMode.SrvFoc)
        servo.set_subdivisions(64)
        servo.set_working_current(2400)
        servo.emergency_stop_motor()

    # STATE UPDATES

    def servo_get_position(self, servo):
        enc_val = servo.read_encoder_value_carry().get("value", 0)
        frac = enc_val / 0x4000
        if frac > 0.5:
            frac = frac - 1
        return float(frac * 2 * math.pi)

    def servo_get_speed(self, servo):
        spd_val = servo.read_motor_speed()
        return float(spd_val / 60 * (2 * math.pi))

    def update_state(self):
        position = [self.servo_get_position(servo) for servo in self.servos]
        velocity = [self.servo_get_speed(servo) for servo in self.servos]

        msg = JointState()
        msg.position = position
        msg.velocity = velocity
        msg.name = [f"joint{i}" for i in range(self.JOINT_CNT)]

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    # CALLBACKS

    def on_command(self, msg):
        if self.mode == 0:
            return

        elif self.mode == 1:
            self.on_position_target(msg.position)

        elif self.mode == 2:
            self.on_velocity_target(msg.velocity)

    def on_position_target(self, position):
        for servo, pos in zip(self.servos, position):
            self.servo_move(servo, pos)

    def on_velocity_target(self, velocity):
        for servo, vel in zip(self.servos, velocity):
            self.servo_speed_mode(servo, vel)

    def on_change_mode(self, request, response):
        if request.mode >= 3 or request.mode < 0:
            response.success = False
        else:
            self.mode = request.mode
            response.success = True
            for servo in self.servos:
                servo.emergency_stop_motor()
        return response

    def on_reset_axis(self, request, response):
        for servo in self.servos:
            servo.set_current_axis_to_zero()
        response.success = True
        return response

    # SERVO COMMANDS
    def servo_move(self, servo, angle):
        if servo.is_motor_running():
            servo.emergency_stop_motor()
            return
        position = int(0x4000 * angle / (2 * math.pi))
        rpm = 60
        acc = 10
        servo.run_motor_absolute_motion_by_axis(rpm, acc, position)

    def servo_speed_mode(self, servo, speed):
        speed_rpm = int(speed / (2 * math.pi) * 60)
        servo.run_motor_in_speed_mode(Direction.CCW if speed_rpm > 0 else Direction.CW, min(abs(speed_rpm), 3000), 0)


def main(args=None):
    rclpy.init(args=args)

    servo_driver = ServoDriver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
