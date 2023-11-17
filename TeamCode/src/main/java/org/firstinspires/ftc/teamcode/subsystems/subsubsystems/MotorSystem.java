package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class MotorSystem {

    /* private final double minPosition, maxPosition, minSpeed, maxSpeed;
    private final DoubleSupplier positionSup;*/

    private double last_speed = 0;
    private double last_time = System.nanoTime() / 1000000000.0;

    private boolean position_mode = true;
    private double target_value;

    private final MotorEx[] motors;

    private static ArrayList<String> registered_systems = new ArrayList<>();

    public MotorSystem(HardwareMap map, Telemetry telemetry, String motor, boolean invert, boolean brake) {
        this(map, telemetry, new String[] { motor }, new boolean[] { invert }, brake);
    }

    public MotorSystem(HardwareMap map, Telemetry telemetry, String[] motor_names, boolean[] invert_motors, boolean brake) {
        motors = new MotorEx[motor_names.length];

        for (int i = 0; i < motors.length; i += 1) {
            motors[i] = new MotorEx(map, motor_names[i]);
            motors[i].setInverted(invert_motors[i]);
            motors[i].setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
            motors[i].setRunMode(Motor.RunMode.RawPower);
        }

        if (!registered_systems.contains(motor_names[0])) {
            registered_systems.add(motor_names[0]);
            motors[0].stopAndResetEncoder();
        }
    }

    public double getPower() {
        return motors[0].get();
    }

    public double getPosition() {
        return motors[0].getCurrentPosition();
    }

    public void set(double speed) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(speed);
        }
    }

    public void stop() {
        set(0);
    }

    /*
    public void update() { // call this during periodic()
        double time = System.nanoTime() / 1000000000.0 - last_time;
        last_time = System.nanoTime() / 1000000000.0;
        if (position_mode) {
            last_speed = 0;
            servo.setPosition(angleToPosition(target_value));
        } else {
            last_speed = Math.max(last_speed - max_acceleration * time, Math.min(last_speed + max_acceleration * time, target_value));
            if (getAngle() + last_speed * time >= maxAngle) {
                last_speed = 0;
                servo.setPosition(angleToPosition(maxAngle));;
            } else if (getAngle() + last_speed * time <= minAngle) {
                last_speed = 0;
                servo.setPosition(angleToPosition(minAngle));
            } else {
                servo.setPosition(angleToPosition(getAngle() + last_speed * time));
            }
        }
    } */
}
