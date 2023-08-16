package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

public final class Constants {

    /* General Constants */

    public static final double trigger_deadzone = 0.1,
                               trigger_exponent = 1.5,
                               consecutive_trigger_time = 0.3;

    public static double normalize_joystick(double val) {
        if (Math.abs(val) < trigger_deadzone) {
            return 0;
        } else if (val > 0) {
            return Math.pow((val - trigger_deadzone) / (1 - trigger_deadzone), trigger_exponent);
        } else {
            return -Math.pow((val + trigger_deadzone) / (trigger_deadzone - 1), trigger_exponent);
        }
    }

    public static String round(double d, int num_places) {
        return "" + Math.round(d * Math.pow(10, num_places)) / Math.pow(10, num_places);
    }

    /* Mecanum Constants */

    public static final String[] wheel_names = new String[]{
        "rightFront",
        "rightBack",
        "leftBack",
        "leftFront"
    }; // ordered by right front, right back, left back, left front

    public static final double strafe = 0.95, // ratio of strafe speed vs forward speed; should be less than one
                               calibration_time = 0.5,
                               yaw_tolerance = 2,
                               min_yaw_correction_power = 0.02,
                               max_yaw_correction_power = 1,
                               yaw_p = 0.035,
                               yaw_e = 1.5,
                               position_calibration_time = 1,
                               position_tolerance = 1.5,
                               min_position_correction_power = 0.01,
                               max_position_correction_power = 0.8,
                               position_p = 0.33, // full power for 4 inches away
                               position_e = 1,
                               distance_weight = 1,
                               turning_weight = 1; // don't turn as much as we go forward

    public static final boolean invert_imu = true, // We want the heading to increase clockwise
                                brake = true;

    /** Normalizes to between -180 and +180 degrees */
    public static double normalize_angle(double degrees) {
        if (degrees < 0) return ((degrees - 180) % 360 + 180);
        return ((degrees + 180) % 360 - 180);
    }

    public static double getCorrection(double error, double p, double e, double min_power, double max_power) {
        double correction = error * p;
        if ((Math.abs(correction) < max_power) && (error * p != 0)) {
            correction *= Math.pow(Math.abs(error * p) / max_power, e - 1);
        }
        if (Math.abs(correction) < min_power) return 0;
        return correction;
    }

    public static double vectorToAngle(double x, double y) {
        if (y == 0) {
            if (x < 0) {
                return -90;
            } else {
                return 90;
            }
        }
        return Math.atan(x / y) * 180.0 / Math.PI + (y > 0 ? 0 : 180) * (x > 0 ? 1 : -1);
    }

    /* Odometry Constants */
    public static final String leftEncoderMotor = "rightFront",
                               rightEncoderMotor = "rightBack",
                               horizontalEncoderMotor = "leftBack";

    public static final double parallel_encoders_width = 12,
                               horizontal_encoder_offset = 12;

    /* Derived Constants */
    public static final double // degrees_per_inch = 360.0 / Math.PI / track_width,
                               odometry_degrees_per_inch = 360.0 / Math.PI / parallel_encoders_width,
                               odometry_horizontal_inches_per_degree = horizontal_encoder_offset * Math.PI / 180.0;

    public static final int[] encoderIndices = {
        Arrays.asList(wheel_names).indexOf(leftEncoderMotor),
        Arrays.asList(wheel_names).indexOf(rightEncoderMotor),
        Arrays.asList(wheel_names).indexOf(horizontalEncoderMotor)
    };
}
