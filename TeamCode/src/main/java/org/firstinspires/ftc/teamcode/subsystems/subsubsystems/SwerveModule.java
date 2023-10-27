package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Variables;

public class SwerveModule {

    private final MotorEx driving_motor;
    private final CRServo angle_motor; // continuous servos are treated as dc motors in code

    private final PIECalculator pieCalculator;

    private final double[] turning_vector;

    private final double min_power = 0.1, max_power = 0.95, min_angular_error = 60;

    public SwerveModule(HardwareMap map, String motor_name, String servo_name, double x, double y) {
        this(map, motor_name, servo_name, x, y, true);
    }

    public SwerveModule(HardwareMap map, String motor_name, String servo_name, double x, double y, boolean brake) {
            // x and y just have to be ratios, they don't have to be in any specific units
            // distance of wheel to center of rotation; positive x is right, positive y is forward
        driving_motor = new MotorEx(map, motor_name);
        angle_motor = new CRServo(map, servo_name);

        driving_motor.setInverted(false);
        driving_motor.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        // driving_motor.setRunMode(Motor.RunMode.RawPower);

        angle_motor.setInverted(false); // we might want to put something in variables to carry information to teleop
        if (Variables.off) {
            driving_motor.stopAndResetEncoder();
        }

        pieCalculator = new PIECalculator(
            0.02, 1, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
            0.05, 0.95, 10, 0.25, 10, () -> driving_motor.getCurrentPosition() * 45.0 / 1024 // 4096 ticks per revolution
        );
        pieCalculator.disable_limiting();

        double distance = Math.sqrt(x * x + y * y);
        turning_vector = new double[] { // for turning right
            y / distance,
            -x / distance
        };
    }

    public double getModuleAngle() {
        return pieCalculator.getCurrentPosition();
    }

    public double getMagnitude(double strafe, double forward, double turn) {
        return Math.sqrt(
            (strafe + turn * turning_vector[0]) * (strafe + turn * turning_vector[0]) +
            (forward + turn * turning_vector[1]) * (forward + turn * turning_vector[1])
        );
    }

    public void setModule(double strafe, double forward, double turn) {
        setModule(
            (strafe + turn * turning_vector[0]),
            (forward + turn * turning_vector[1])
        );
    }

    public void setModule(double x, double y) {
        double magnitude = Math.sqrt(x * x + y * y);
        if (magnitude < min_power) {
            angle_motor.set(0);
            driving_motor.set(0);
            return; // whatever we want the minimum magnitude to be
        }

        double target_angle = vectorToAngle(x, y);

        if (Math.abs(normalize_angle(target_angle - getModuleAngle())) > 90) {
            target_angle += 180 * (target_angle > 0 ? -1 : 1);
            magnitude *= -1;
        }

        target_angle = getModuleAngle() - normalize_angle(getModuleAngle() - target_angle);

        pieCalculator.setTarget(target_angle);

        angle_motor.set(pieCalculator.getPower(0));

        if (Math.abs(normalize_angle(target_angle - getModuleAngle())) > min_angular_error) magnitude = 0;

        driving_motor.set(magnitude);
    }

    public double getTargetAngle() {
        return pieCalculator.getTarget();
    }
}
