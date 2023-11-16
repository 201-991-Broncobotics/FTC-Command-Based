package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanOdometry extends Mecanum {

    protected final MotorEx.Encoder[] deadwheels;

    public MecanOdometry(HardwareMap map, Telemetry telemetry, String[] motor_names, boolean[] inverted,
                   double starting_x, double starting_y, double starting_angle, boolean invert_imu,
                   RevHubOrientationOnRobot.LogoFacingDirection logo_direction,
                   RevHubOrientationOnRobot.UsbFacingDirection usb_direction, String[] deadwheel_names
    ) {
        super(
            map, telemetry, motor_names, inverted, starting_x, starting_y, starting_angle, invert_imu,
            logo_direction, usb_direction
        );

        deadwheels = new MotorEx.Encoder[] {
            new MotorEx(map, deadwheel_names[0]).encoder,
            new MotorEx(map, deadwheel_names[2]).encoder,
            new MotorEx(map, deadwheel_names[1]).encoder
        };

        for (int i = 0; i < 4; i++) {
            deadwheels[i].setDirection(MotorEx.Direction.FORWARD);
            deadwheels[i].reset();
        }

        resetEncoders();
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            deadwheels[i].reset();
        }
    }
}
