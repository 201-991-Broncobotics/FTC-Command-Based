package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

public class Mecanum extends DriveSubsystemBase {

    protected final MotorEx[] motors;
    protected final MotorEx.Encoder[] encoders;

    private final double strafe = 1;
        // if strafing 10 in goes up 105 on encoders, but going forward 10 in goes up 100, then strafe is 1.05

    protected final boolean brake = true;

    /** ordered by right front, right back, left back, left front; x and y in inches, angle in degrees clockwise */
    public Mecanum(HardwareMap map, Telemetry telemetry, String[] motor_names, boolean[] inverted,
                   double starting_x, double starting_y, double starting_angle, boolean invert_imu,
                   RevHubOrientationOnRobot.LogoFacingDirection logo_direction,
                   RevHubOrientationOnRobot.UsbFacingDirection usb_direction
    ) {
        super(
            map, telemetry, invert_imu, logo_direction, usb_direction, starting_x, starting_y,
            starting_angle, true, 0.5,0.1,
            0.25, 0.1, 1.05, 0.5,
            0, 0, 0, 1, 0.95
        );

        motors = new MotorEx[] {
            new MotorEx(map, motor_names[0], MotorEx.GoBILDA.RPM_312),
            new MotorEx(map, motor_names[1], MotorEx.GoBILDA.RPM_312),
            new MotorEx(map, motor_names[2], MotorEx.GoBILDA.RPM_312),
            new MotorEx(map, motor_names[3], MotorEx.GoBILDA.RPM_312)
        };

        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(brake ? MotorEx.ZeroPowerBehavior.BRAKE : MotorEx.ZeroPowerBehavior.FLOAT);
            motors[i].setRunMode(MotorEx.RunMode.RawPower);
            motors[i].setInverted((i < 2) == !inverted[i]); // invert if i = 0, 1
        }

        encoders = new MotorEx.Encoder[] {
            motors[0].encoder,
            motors[1].encoder,
            motors[2].encoder,
            motors[3].encoder
        };

        for (int i = 0; i < 4; i++) {
            encoders[i].setDirection(MotorEx.Direction.FORWARD);
            encoders[i].reset();
        }

        resetEncoders();
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            encoders[i].reset();
        }
    }

    public double[] getEncoderInches() {
        double wheel_diameter = 96 / 25.4; // in inches, roughly 3.7795275591
        return new double[] {
            encoders[0].getRevolutions() * Math.PI * wheel_diameter,
            encoders[1].getRevolutions() * Math.PI * wheel_diameter,
            encoders[2].getRevolutions() * Math.PI * wheel_diameter,
            encoders[3].getRevolutions() * Math.PI * wheel_diameter
        };
    }

    @Override
    public void brake() {
        for (int i = 0; i < 4; i++) {
            motors[i].set(0);
        }
    }

    @Override
    public double get_max_power(double offset, double distance_factor, double turning_factor) {
        double maximum = 0;
        for (int i = 0; i < 4; i++) {
            maximum = Math.max(maximum,
                turning_factor * ((i > 1) ? 1 : -1) + distance_factor * (Math.cos(offset * Math.PI / 180.0) + Math.sin(offset * Math.PI / 180.0) * (i % 2 == 1 ? 1 : -1) * strafe)
            );
        }
        return maximum;
    }

    @Override
    public void power_motors(double offset, double distance_factor, double turning_factor) {
        for (int i = 0; i < 4; i++) {
            motors[i].set(
                turning_factor * ((i > 1) ? 1 : -1) + distance_factor * (Math.cos(offset * Math.PI / 180.0) + Math.sin(offset * Math.PI / 180.0) * (i % 2 == 1 ? 1 : -1) * strafe)
            );
        }
    }

    @Override
    public double getForwardInches() {
        double[] encoderInches = getEncoderInches();
        return -(encoderInches[0] + encoderInches[1] + encoderInches[2] + encoderInches[3]) / 4.0;
    }

    @Override
    public double getStrafeInches() {
        double[] encoderInches = getEncoderInches();
        resetEncoders(); // reset for getStrafeInches but NOT for getForwardInches
        return (encoderInches[0] - encoderInches[1] + encoderInches[2] - encoderInches[3]) / 4.0 / strafe;
    }

    @Override
    public void periodic() {
        base_periodic_loop();
    }
}
