package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;

public class Swerve extends DriveSubsystemBase {

    private final SwerveModule[] modules;

    /** ordered by right front, right back, left back, left front */
    public Swerve(HardwareMap map, Telemetry telemetry, String[] motor_names, String[] servo_names,
                  double width, double length, boolean invert_imu,
                  RevHubOrientationOnRobot.LogoFacingDirection logo_direction,
                  RevHubOrientationOnRobot.UsbFacingDirection usb_direction
    ) {
        super(
            map, telemetry, invert_imu, logo_direction, usb_direction, 0, 0, 0,
            true, 0.5,0.1, 0.5,
            0.025, 1.05, 0.5, 0,
            0, 0, 1, 0.95
        );

        modules = new SwerveModule[] {
            new SwerveModule(map, motor_names[0], servo_names[0], width, length),
            new SwerveModule(map, motor_names[1], servo_names[1], width, -length),
            new SwerveModule(map, motor_names[2], servo_names[2], -width, -length),
            new SwerveModule(map, motor_names[3], servo_names[3], -width, length)
        };
        // length is 9.133858; width is 12.913386
    } // ordered by right front, right back, left back, left front
      // MAKE SURE all the wheels are pointing with the gear facing the direction considered as "right"

    @Override
    public void brake() {
        for (int i = 0; i < 4; i++) modules[i].setModule(0, 0);
    }

    @Override
    public double get_max_power(double offset, double distance_factor, double turning_factor) {
        double maximum = 0;
        for (int i = 0; i < 4; i++) {
            maximum = Math.max(maximum, modules[i].getMagnitude(
                distance_factor * Math.sin(offset * Math.PI / 180.0),
                distance_factor * Math.cos(offset * Math.PI / 180.0),
                turning_factor
            ));
        }
        return maximum;
    }

    @Override
    public void power_motors(double offset, double distance_factor, double turning_factor) {
        for (int i = 0; i < 4; i++) { // this must be replaced with 4 :)
            modules[i].setModule(
                distance_factor * Math.sin(offset * Math.PI / 180.0),
                distance_factor * Math.cos(offset * Math.PI / 180.0),
                turning_factor
            );
        }
    }

    @Override
    public double getForwardInches() {
        return 0;
    }

    @Override
    public double getStrafeInches() {
        return 0;
    }

    @Override
    public void periodic() {
        base_periodic_loop();
    }

}
