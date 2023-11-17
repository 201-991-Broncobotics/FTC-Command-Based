package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class ServoEx {

    private final Servo servo;
    private final double zero, minAngle, maxAngle, range, inverted, max_acceleration;
    private final DoubleSupplier positionSup;

    private double last_speed = 0;
    private double last_time = System.nanoTime() / 1000000000.0;

    private boolean position_mode = true;
    private double target_value;

    public ServoEx(HardwareMap map, String name, double zero, double minAngle, double maxAngle,
                   double starting_angle, double max_acceleration, double range, boolean inverted) {
        servo = map.get(Servo.class, name);
        servo.setDirection(Servo.Direction.FORWARD);

        this.zero = zero; // whatever the servo reads at the "zero" angle; will be between 0 and 1
        // raw reading, regardless of inversions and whatnot
        // for example, vertical might be at servo position 0.3, and we want vertical to be
        // considered 0; this variable should be set to 0.3 in this case
        this.minAngle = minAngle; // whatever angle we want the minimum to be (not necessarily physical minimum)
        this.maxAngle = maxAngle; // assuming the zero position is at 0 degrees
        this.max_acceleration = max_acceleration; // in degrees
        this.range = range; // range of servo; will either be 300 or 1800 degrees
        // You can use any units you wish to, you just have to be consistent with it

        this.inverted = inverted ? -1 : 1;

        target_value = starting_angle; // what we want to start at, not necessarily what we actually start at

        positionSup = () -> ((servo.getPosition() - zero) * range) * this.inverted;
    }

    private double angleToPosition(double angle) {
        return Math.max(minAngle, Math.min(maxAngle, angle)) / range * inverted + zero;
    }

    public double positionToAngle(double position) {
        return ((position - zero) * range) * inverted;
    }

    public double getRawPosition() {
        return servo.getPosition();
    }

    public double getAngle() {
        return positionSup.getAsDouble();
    }

    public void setAngle(double angle) {
        position_mode = true;
        last_speed = 0;
        target_value = angle;
    }

    public void rotateByAngle(double angle) {
        setAngle(angle + getAngle());
    }

    public void set(double speed) { // speed is units per second
        position_mode = false;
        target_value = speed;
    }

    public double get() {
        return last_speed;
    }

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
                servo.setPosition(angleToPosition(maxAngle));
            } else if (getAngle() + last_speed * time <= minAngle) {
                last_speed = 0;
                servo.setPosition(angleToPosition(minAngle));
            } else {
                servo.setPosition(angleToPosition(getAngle() + last_speed * time));
            }
        }
    }
}
