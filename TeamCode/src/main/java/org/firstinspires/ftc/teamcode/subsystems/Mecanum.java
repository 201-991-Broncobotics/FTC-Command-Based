package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Mecanum extends SubsystemBase { // uses the control hub's IMU to keep a constant heading
    protected final Telemetry telemetry;

    protected final MotorEx[] motors; // right front, right back, left back, left front
    protected final Motor.Encoder[] encoders; // same as above

    protected final IMU imu;
    protected double imu_zero, last_time, target_heading;
    protected double last_translation_time, target_x, target_y;

    protected boolean fieldCentric = true;

    protected final double[] pose = new double[3];
    // x, y, theta; for theta, 0 means straight ahead, positive angle means rotated clockwise in degrees

    /** x and y in inches, angle in degrees clockwise */
    public Mecanum(HardwareMap map, Telemetry telemetry, double starting_x, double starting_y, double starting_angle) {
        this.telemetry = telemetry;

        motors = new MotorEx[] {
                new MotorEx(map, wheel_names[0], Motor.GoBILDA.RPM_312),
                new MotorEx(map, wheel_names[1], Motor.GoBILDA.RPM_312),
                new MotorEx(map, wheel_names[2], Motor.GoBILDA.RPM_312),
                new MotorEx(map, wheel_names[3], Motor.GoBILDA.RPM_312)
        };

        for (int i = 0; i < 4; i++) {
            motors[i].setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
            motors[i].setRunMode(Motor.RunMode.RawPower);
            motors[i].setInverted(i < 2); // invert if i = 0, 1
        }

        encoders = new Motor.Encoder[] {
                motors[0].encoder,
                motors[1].encoder,
                motors[2].encoder,
                motors[3].encoder
        };

        for (int i = 0; i < 4; i++) {
            encoders[i].setDirection(Motor.Direction.FORWARD);
            encoders[i].reset();
        }


        imu = map.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT // Directions of Control Hub
        )));

        resetPosition(starting_x, starting_y);
        resetIMU(starting_angle);

        resetEncoders();
    }

    public void resetIMU(double angle) {
        imu.resetYaw();
        imu_zero = angle * (invert_imu ? 1 : -1);
        last_time = 0;
        pose[2] = angle; // just as a hard reset, in theory it shouldn't be necessary
        target_heading = angle;
    }

    public void resetPosition(double x, double y) {
        last_translation_time = 0;
        pose[0] = x;
        pose[1] = y;
        target_x = x;
        target_y = y;
    }

    public void setTargetHeading(double target) {
        last_time = 0;
        target_heading = target;
    }

    public void setTargetPosition(double x, double y) {
        last_translation_time = 0;
        target_x = x;
        target_y = y;
    }

    public void resetOdometry() {
        resetTargetHeading();
        resetTargetPosition();
    }

    public void resetTargetHeading() {
        last_time = System.currentTimeMillis() / 1000.0;
        target_heading = pose[2];
    }

    public void resetTargetPosition() {
        last_translation_time = System.currentTimeMillis() / 1000.0;
        target_x = pose[0];
        target_y = pose[1];
    }

    public double getAngle() {
        return normalize_angle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - imu_zero) * (invert_imu ? -1 : 1);
    }

    public double[] getOdometry() {
        return pose;
    }

    /** Increases it clockwise from the current heading, not from the current target */
    public void changeHeading(double degrees) {
        setTargetHeading(getAngle() + normalize_angle(degrees));
    }

    public void changeXY(double delta_x, double delta_y) {
        setTargetPosition( pose[0] + delta_x, pose[1] + delta_y);
    }

    public double getError() {
        return normalize_angle(target_heading - getAngle());
    }

    public double getPositionError() {
        return Math.sqrt(
                (target_x - pose[0]) * (target_x - pose[0]) + (target_y - pose[1]) * (target_y - pose[1])
        );
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

    public void drive(double strafe_factor, double forward_factor, double turning_factor) {
        drive(strafe_factor, forward_factor, turning_factor, fieldCentric);
    }

    public void toggleDriveMode() {
        fieldCentric = !fieldCentric;
    }

    public void brake() {
        for (int i = 0; i < 4; i++) {
            motors[i].stopMotor();
        }
    }

    public void drive(double strafe_factor, double forward_factor, double turning_factor, boolean fieldCentric) {

        /* Calculations */

        double offset = vectorToAngle(strafe_factor, forward_factor);
        double distance_factor = Math.sqrt(strafe_factor * strafe_factor + forward_factor * forward_factor);

        /* Position Correction */

        if (Math.abs(distance_factor) < 0.01) {
            if (System.currentTimeMillis() / 1000.0 - last_translation_time < position_calibration_time) {
                target_x = pose[0];
                target_y = pose[1];
            } else {
                Translation2d position_error = new Translation2d(
                        target_x - pose[0],
                        target_y - pose[1]
                );
                double norm = getCorrection(position_error.getNorm(), position_p, position_e, min_position_correction_power, max_position_correction_power);
                position_error = position_error.times(norm / position_error.getNorm());
                distance_factor = position_error.getNorm();
                offset = vectorToAngle(position_error.getX(), position_error.getY());
            }
        } else {
            target_x = pose[0];
            target_y = pose[1];
            last_time = System.currentTimeMillis() / 1000.0;
        }

        /* Heading Correction */

        double current_heading = getAngle(); // positive --> clockwise
        if (Math.abs(turning_factor) < 0.01) {
            if (System.currentTimeMillis() / 1000.0 - last_time < calibration_time) {
                target_heading = current_heading;
            } else {
                turning_factor = getCorrection(getError(), yaw_p, yaw_e, min_yaw_correction_power, max_yaw_correction_power);
            }
        } else {
            target_heading = current_heading;
            last_time = System.currentTimeMillis() / 1000.0;
        }

        /* Field Centric */

        if (fieldCentric) {
            offset -= getAngle();
        }

        distance_factor *= distance_weight;
        turning_factor *= turning_weight;

        if (Math.abs(distance_factor) < min_position_correction_power) distance_factor = 0;
        if (Math.abs(turning_factor) < min_yaw_correction_power) turning_factor = 0;

        double[] power = new double[4];
        for (int i = 0; i < 4; i++) {
            power[i] = turning_factor * ((i > 1) ? -1 : 1) - distance_factor * (Math.cos(offset * Math.PI / 180.0) + Math.sin(offset * Math.PI / 180.0) * (i % 2 == 1 ? 1 : -1) / strafe);
        }
        double maximum = Math.max(1, Math.max(Math.max(Math.abs(power[0]), Math.abs(power[1])), Math.max(Math.abs(power[2]), Math.abs(power[3]))));
        for (int i = 0; i < 4; i++) {
            motors[i].set(power[i] / maximum * org.firstinspires.ftc.teamcode.Variables.max_speed);
        }
    }
    public void update_odometry() {
        double previous_angle = pose[2];
        pose[2] = getAngle();
        double[] encoderInches = getEncoderInches(); // gets change in encoders from last reading
        if (encoderInches == null) {
            return;
        }
        resetEncoders();

        /* Here's where we get the math
        RF = -forward + turning + strafe
        RB = -forward + turning - strafe
        LB = -forward - turning + strafe
        LF = -forward - turning - strafe

        RF + RB + LB + LF =-4 * forward

        RF - RB + LB - LF = 4 * strafe

        RF + RB - LB - LF = 4 * turning */

        double forward_inches = -(encoderInches[0] + encoderInches[1] + encoderInches[2] + encoderInches[3]) / 4.0;
        double strafe_inches = (encoderInches[0] - encoderInches[1] + encoderInches[2] - encoderInches[3]) / 4.0;
        double turning_inches = (encoderInches[0] + encoderInches[1] - encoderInches[2] - encoderInches[3]) / 4.0;

        strafe_inches *= strafe;

        double turning_degrees = getAngle() - previous_angle;
        // turning_inches * degrees_per_inch;

        double raw_distance = Math.sqrt(strafe_inches * strafe_inches + forward_inches * forward_inches);

        double relative_angle = vectorToAngle(strafe_inches, forward_inches);
        relative_angle += turning_degrees * 0.5;

        double distance;
        if (Math.abs(turning_degrees) < 0.01) { // assume it's linear

            distance = raw_distance;

        } else {

            double radius = raw_distance * 180 / Math.PI / Math.abs(turning_degrees);
            distance = radius * Math.sqrt(2 - 2 * Math.cos(turning_degrees * Math.PI / 180.0));

        }

        telemetry.addLine("" + round(distance, 2) + " " + round(previous_angle, 2) + " " + round(relative_angle, 2));

        pose[0] += distance * Math.sin((previous_angle + relative_angle) * Math.PI / 180.0);
        pose[1] += distance * Math.cos((previous_angle + relative_angle) * Math.PI / 180.0);
    }

    @Override
    public void periodic() {
        update_odometry();

        telemetry.addData("Current angle", round(getAngle(), 2));
        telemetry.addData("Current angular error", round(getError(), 2));

        telemetry.addData("Odometry position (inches)", "(" + round(pose[0], 2) + ", " + round(pose[1], 2) + ")");
        telemetry.addData("Odometry angle (degrees)", round(pose[2], 2));

        telemetry.addData("Drive mode", fieldCentric ? "Field Centric" : "Robot Centric");
    }
}