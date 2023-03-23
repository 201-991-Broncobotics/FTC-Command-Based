package org.firstinspires.ftc.teamcode.subsystems.examples;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

public class Mecanum_Odometry extends Mecanum { // has the same deadwheel setup as the roadrunner setup

    private final double time_inverval;
    private double update_with_IMU;

    public Mecanum_Odometry(HardwareMap map, Telemetry telemetry, double starting_x, double starting_y, double starting_angle, double time_interval) {
        super(map, telemetry, starting_x, starting_y, starting_angle);

        for (int i = 0; i < 4; i++) {
            motors[i].setRunMode(Motor.RunMode.RawPower);
            motors[i].encoder.setDistancePerPulse(1.0 / 8192.0);
        }
        // using IMU makes the position more accurate, but if we update too often we lose accuracy
        // to limit the inaccuracies, we only reset to the IMU's heading at specific intervals, not every loop

        // might have to invert some encoders

        this.time_inverval = time_interval;
        update_with_IMU = System.currentTimeMillis() / 1000.0 + time_inverval;
    }

    // In this class, we only have to override the odometry methods - not the driving or PID methods

    public double getCalculatedAngle() {
        return pose[2];
    }

    @Override
    public void resetEncoders() {
        for (int i : encoderIndices) encoders[i].reset();
    }

    @Override
    public double[] getEncoderInches() {
        double wheel_diameter = 35 / 25.4; // in inches, roughly 3.7795275591
        return new double[] {
            encoders[encoderIndices[0]].getDistance() * Math.PI * wheel_diameter,
            encoders[encoderIndices[1]].getDistance() * Math.PI * wheel_diameter,
            encoders[encoderIndices[2]].getDistance() * Math.PI * wheel_diameter
        };
    }

    @Override
    public void update_odometry() {
        // use encoders to update it instead
        if (System.currentTimeMillis() > update_with_IMU) {
            update_with_IMU = System.currentTimeMillis() / 1000.0 + time_inverval;
            pose[2] = getAngle();
        }

        double[] encoderInches = getEncoderInches(); // gets change in encoders from last reading
        resetEncoders();


        /* Here's where we get the math
        L = forward + turning
        R = forward - turning
        H =         - turning + strafe

        However, the turning has to be multiplied by factors.
            For L and R, it's multiplied by 360 * / pi / L = odometry_degrees_per_inch
            For H, it's multiplied by 180 / (pi * dist) = odometry_horizontal_inches_per_degree
        L + R = 2 * forward

        We can use that to figure out turning, which we can use to figure out strafe */

        double forward_inches = (encoderInches[0] + encoderInches[1]) / 2.0;
        double turning_degrees = (encoderInches[0] - encoderInches[1]) / 2.0 * odometry_degrees_per_inch;
        double strafe = encoderInches[3] + turning_degrees * odometry_horizontal_inches_per_degree;
    }
}
