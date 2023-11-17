package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.normalize_angle;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.SwerveModule;

public class Swerve extends DriveSubsystemBase {

    private final SwerveModule[] modules;
    private final MotorEx.Encoder encoder;

    private static boolean reset_modules = true;

    /** ordered by right front, right back, left back, left front */
    public Swerve(HardwareMap map, Telemetry telemetry, String[] motor_names, String[] servo_names,
                  double width, double length, boolean invert_imu,
                  RevHubOrientationOnRobot.LogoFacingDirection logo_direction,
                  RevHubOrientationOnRobot.UsbFacingDirection usb_direction
    ) {
        this(
                map, telemetry, motor_names, servo_names, width, length, invert_imu, logo_direction,
                usb_direction, motor_names[0]
        );
    }

    public Swerve(HardwareMap map, Telemetry telemetry, String[] motor_names, String[] servo_names,
                  double width, double length, boolean invert_imu,
                  RevHubOrientationOnRobot.LogoFacingDirection logo_direction,
                  RevHubOrientationOnRobot.UsbFacingDirection usb_direction, String encoder_name
    ) {
        super(
                map, telemetry, invert_imu, logo_direction, usb_direction, 0, 0, 0,
                true, 0.5,0.1, 0.25,
                0.025, 1.05, 0.5, 0,
                0, 0, 1, 0.95
        );

        modules = new SwerveModule[] {
                new SwerveModule(map, motor_names[0], servo_names[0], width, length, reset_modules),
                new SwerveModule(map, motor_names[1], servo_names[1], width, -length, reset_modules),
                new SwerveModule(map, motor_names[2], servo_names[2], -width, -length, reset_modules),
                new SwerveModule(map, motor_names[3], servo_names[3], -width, length, reset_modules)
        };

        reset_modules = false;

        encoder = new MotorEx(map, encoder_name).encoder;
    } // ordered by right front, right back, left back, left front
    // MAKE SURE all the wheels are pointing with the gear facing the direction considered as "right"

    public void resetEncoder() {
        encoder.reset();
    }

    public double readEncoderDistance() { // 187/100548 revolutions per tick; circumference = 86pi mm
        return encoder.getPosition() * 187.0 / 100548.0 * Math.PI * 86.0 / 25.4;
    }

    public void setModuleAngles(double target_angle) {
        for (int i = 0; i < 4; i++) {
            modules[i].setModuleAngle(target_angle);
        }
    }

    public double getModuleError(double target_angle) {
        double error = 0;
        for (int i = 0; i < 4; i++) {
            error = Math.max(error, Math.abs(normalize_angle(target_angle - modules[i].getModuleAngle())));
        }
        return error;
    }

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
        boolean stop = false;
        for (int i = 0; i < 4; i++) { // this must be replaced with 4 :)
            stop |= modules[i].setModule(
                    distance_factor * Math.sin(offset * Math.PI / 180.0),
                    distance_factor * Math.cos(offset * Math.PI / 180.0),
                    turning_factor
            );
        }

        if (stop) {
            for (int i = 0; i < 4; i++) {
                modules[i].stopDriving();
            }
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

