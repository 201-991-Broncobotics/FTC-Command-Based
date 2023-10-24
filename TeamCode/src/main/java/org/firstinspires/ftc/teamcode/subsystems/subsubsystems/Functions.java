package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class Functions {

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
}
