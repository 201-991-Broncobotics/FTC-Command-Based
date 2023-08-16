package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class ServoEx {

    private final Servo servo;
    private final double zero, minAngle, maxAngle, range, inverted;
    private final DoubleSupplier positionSup;

    private boolean position_mode = true;
    private double target_value = 0; // either speed or position depending on what mode we're in
    private double last_speed = 0;
    private double last_time = System.nanoTime() / 1000000000.0;

    public ServoEx(HardwareMap map, String name, double zero, double minAngle, double maxAngle,
                   double range, boolean inverted) {
        servo = map.get(Servo.class, name);

        this.zero = zero; // whatever the servo reads at the "zero" position; will be between 0 and 1
            // raw reading, regardless of inversions and whatnot
        this.minAngle = minAngle; // whatever angle we want the minimum to be (not necessarily physical minimum)
        this.maxAngle = maxAngle; // assuming the zero position is at 0 degrees
        this.range = range; // range of servo; will either be 300 or 1800 degrees
        // You can use any units you wish to, you just have to be consistent with it

        this.inverted = inverted ? -1 : 1;

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
        target_value = angleToPosition(angle);
    }

    public void rotateByAngle(double angle) {
        setAngle(angle + getAngle());
    }

    public double get() {
        if (position_mode) {
            return 0;
        }
        return last_speed;
    }

    public void set(double speed) { // speed is units per second
        position_mode = false;
        target_value = speed; // perhaps clip to maximum speed but whatever
    }

    public void update() { // call this during periodic()
        if (position_mode) {
            servo.setPosition(target_value);
            last_speed = 0;
        } else {
            double time = System.nanoTime() / 1000000000.0 - last_time;
            double speed = Math.max(last_speed - 10 * time, Math.min(last_speed + 10 * time, target_value)); // whatever accel is
            if ((getAngle() + speed * time >= maxAngle) || (getAngle() + speed * time <= minAngle)) {
                last_speed = 0;
                position_mode = true;
            }
            servo.setPosition(angleToPosition(getAngle() + speed * time));
        }
        last_time = System.nanoTime() / 1000000000.0;
    }
}