package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Variables;

public abstract class DriveSubsystemBase extends SubsystemBase {

    protected final Telemetry telemetry;

    private final IMU imu;
    private final boolean invert_imu;
    private double imu_zero, last_time, target_heading;

    private double last_translation_time, target_x, target_y;

    private final double[] pose;
    // x, y, theta; for theta, 0 means straight ahead, positive angle means rotated clockwise in degrees

    private boolean fieldCentric;

    private final double heading_calibration_time,
        min_heading_correction_power,
        max_heading_correction_power,
        heading_p,
        heading_e,
        position_calibration_time,
        min_position_correction_power,
        max_position_correction_power,
        position_p,
        position_e,
        maximum_motor_power; // don't turn as much as we go forward

    /** x and y in inches, angle in degrees clockwise */
    public DriveSubsystemBase(HardwareMap map, Telemetry telemetry, boolean invert_imu,
        RevHubOrientationOnRobot.LogoFacingDirection logo_direction,
        RevHubOrientationOnRobot.UsbFacingDirection usb_direction,
        double starting_x, double starting_y, double starting_angle, boolean fieldCentric,
        double heading_calibration_time, double min_heading_correction_power,
        double max_heading_correction_power, double heading_p, double heading_e, double position_calibration_time,
        double min_position_correction_power, double max_position_correction_power,
        double position_p, double position_e, double maximum_motor_power
    ) {

        this.telemetry = telemetry;

        imu = map.get(IMU.class, "imu");
        this.invert_imu = invert_imu;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo_direction, usb_direction)));

        pose = new double[] {
            starting_x, starting_y, starting_angle
        };

        resetPosition(starting_x, starting_y);
        resetHeading(starting_angle);

        this.fieldCentric = fieldCentric;

        this.heading_calibration_time = heading_calibration_time;
        this.min_heading_correction_power = min_heading_correction_power;
        this.max_heading_correction_power = max_heading_correction_power;
        this.heading_p = heading_p;
        this.heading_e = heading_e;
        this.position_calibration_time = position_calibration_time;
        this.min_position_correction_power = min_position_correction_power;
        this.max_position_correction_power = max_position_correction_power;
        this.position_p = position_p;
        this.position_e = position_e;
        this.maximum_motor_power = maximum_motor_power;
    }

    public final double getHeading() {
        return normalize_angle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - imu_zero) * (invert_imu ? -1 : 1);
    }

    public final void resetHeading(double angle) {
        imu.resetYaw();
        imu_zero = angle * (invert_imu ? 1 : -1);
        last_time = 0;
        target_heading = angle;
        pose[2] = angle; // just as a hard reset, in theory it shouldn't be necessary
    }

    public final void setTargetHeading(double target) {
        last_time = 0;
        target_heading = target;
    }

    public final void resetTargetHeading() {
        last_time = System.currentTimeMillis() / 1000.0;
        target_heading = pose[2];
    }

    /* Odometry */

    public final void resetPosition(double x, double y) {
        last_translation_time = 0;
        pose[0] = x;
        pose[1] = y;
        target_x = x;
        target_y = y;
    }

    public final void setTargetPosition(double x, double y) {
        last_translation_time = 0;
        target_x = x;
        target_y = y;
    }

    public final void resetTargetPosition() {
        last_translation_time = System.currentTimeMillis() / 1000.0;
        target_x = pose[0];
        target_y = pose[1];
    }

    public final void resetTargets() {
        resetTargetHeading();
        resetTargetPosition();
    }

    public final double[] getOdometry() {
        return pose;
    }

    /** Increases it clockwise from the current heading, not from the current target */
    public final void changeHeading(double degrees) {
        setTargetHeading(getHeading() + normalize_angle(degrees));
    }

    public final void changeXY(double delta_x, double delta_y) {
        setTargetPosition( pose[0] + delta_x, pose[1] + delta_y);
    }

    public final double getHeadingError() {
        return normalize_angle(target_heading - getHeading());
    }

    public final double getPositionError() {
        return Math.sqrt(
            (target_x - pose[0]) * (target_x - pose[0]) + (target_y - pose[1]) * (target_y - pose[1])
        );
    }

    public final void drive(double strafe_factor, double forward_factor, double turning_factor, double damping) {
        drive(strafe_factor, forward_factor, turning_factor, fieldCentric, damping);
    }

    public final void toggleDriveMode() {
        fieldCentric = !fieldCentric;
    }

    /** Should stop every motor */
    public abstract void brake();

    /** Damping is the dividing factor; ex, if we want to go at 50% speed, set damping to 2. Affects translation, not rotation */
    public final void drive(double strafe_factor, double forward_factor, double turning_factor, boolean fieldCentric, double damping) {

        /* Damping */

        strafe_factor /= damping;
        forward_factor /= damping;

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
                position_error = position_error.times(position_p);
                distance_factor = Math.min(position_error.getNorm(), max_position_correction_power);
                if (distance_factor > 0) distance_factor *= Math.pow(Math.abs(distance_factor) / max_position_correction_power, position_e - 1);
                offset = vectorToAngle(position_error.getX(), position_error.getY());
            }
        } else {
            target_x = pose[0];
            target_y = pose[1];
            last_time = System.currentTimeMillis() / 1000.0;
        }

        /* Heading Correction */

        double current_heading = getHeading(); // positive --> clockwise
        if (Math.abs(turning_factor) < 0.01) {
            if (System.currentTimeMillis() / 1000.0 - last_time < heading_calibration_time) {
                target_heading = current_heading;
            } else {
                turning_factor = getCorrection(getHeadingError(), heading_p, heading_e, min_heading_correction_power, max_heading_correction_power);
            }
        } else {
            target_heading = current_heading;
            last_time = System.currentTimeMillis() / 1000.0;
        }

        /* Field Centric */

        if (fieldCentric) {
            offset -= getHeading();
        }

        double max_power = get_max_power(offset, distance_factor, turning_factor);
        if (max_power >= maximum_motor_power) {
            distance_factor *= maximum_motor_power / max_power;
            turning_factor *= maximum_motor_power / max_power;
        }

        if (Math.abs(distance_factor) < min_position_correction_power) distance_factor = 0;
        if (Math.abs(turning_factor) < min_heading_correction_power) turning_factor = 0;

        power_motors(offset, distance_factor, turning_factor);
    }

    public abstract double get_max_power(double offset, double distance_factor, double turning_factor); // max power going to the 4 wheels

    public abstract void power_motors(double offset, double distance_factor, double turning_factor);

    /* Note these two are NOT final; should be inches since last call */
    public double getForwardInches() {
        return 0;
    }

    public double getStrafeInches() {
        return 0;
    }

    public final void update_odometry() {
        double previous_angle = pose[2];
        pose[2] = getHeading();

        double forward_inches = getForwardInches();
        double strafe_inches = getStrafeInches();

        double turning_degrees = getHeading() - previous_angle;

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

    public final void base_periodic_loop() { // call this periodic() in super() periodic
        update_odometry();

        if (Variables.teleOp) {

            // by the way, the program updates much faster than the telemetry updates :)

            telemetry.addData("Current angle", round(getHeading(), 2));
            telemetry.addData("Current angular error", round(getHeadingError(), 2));

            telemetry.addData("Odometry position (inches)", "(" + round(pose[0], 2) + ", " + round(pose[1], 2) + ")");
            telemetry.addData("Odometry angle (degrees)", round(pose[2], 2));

            telemetry.addData("Drive mode", fieldCentric ? "Field Centric" : "Robot Centric");
        }
    }

    @Override
    public void periodic() {
        base_periodic_loop();
    }
}
