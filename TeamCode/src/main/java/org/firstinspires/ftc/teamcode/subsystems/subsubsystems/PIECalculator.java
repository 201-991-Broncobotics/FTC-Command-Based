package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import java.util.function.DoubleSupplier;

/** Automatically handles acceleration limiting and position control */
public class PIECalculator {

    private final double kP, kE, kI, minPosition, maxPosition, minPower, maxPower, acceleration, calibration_time, tolerance;
    private final DoubleSupplier positionSup;

    private double integral, previous_time, target_position, last_time, last_manually_targeted_position, previous_power;
    private boolean enable_limiting = true; // if we want to be bounded by min/max position. Otherwise we only do PID

    public PIECalculator(double kP, double kE, double kI, double minPosition, double maxPosition,
                         double minPower, double maxPower, double acceleration, double calibration_time,
                         double tolerance, DoubleSupplier positionSup) {
        this.kP = kP;
        this.kE = kE;
        this.kI = kI;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.acceleration = acceleration;
        this.calibration_time = calibration_time;
        this.tolerance = tolerance;

        this.positionSup = positionSup;

        reset();
    }

    public PIECalculator(double kP, double kE, double minPosition, double maxPosition,
                         double minPower, double maxPower, double acceleration, double calibration_time,
                         double tolerance, DoubleSupplier positionSup) {
        this(kP, kE, 0, minPosition, maxPosition, minPower, maxPower, acceleration, calibration_time, tolerance, positionSup);
    }

    /* Accessor Methods */
    public double getMaxPosition() {
        return maxPosition;
    }

    public boolean isWithinTolerance() {
        return Math.abs(target_position - positionSup.getAsDouble()) < tolerance;
    }

    public void setTarget(double target_position, boolean resetIntegral) {
        this.target_position = target_position;
        last_time = 0;
        previous_time = System.currentTimeMillis() / 1000.0;
        if (resetIntegral) integral = 0;
    }

    public void setTarget(double target_position) {
        setTarget(target_position, true);
    }

    public double getTarget() {
        return target_position;
    }

    public void reset() {
        target_position = positionSup.getAsDouble();
        last_manually_targeted_position = target_position;
        last_time = System.currentTimeMillis() / 1000.0;
        previous_time = System.currentTimeMillis() / 1000.0;
        integral = 0;
    }

    private double getCorrection() {
        double delta_time = System.currentTimeMillis() / 1000.0 - previous_time;

        double error = target_position - positionSup.getAsDouble();

        double p = error * kP;

        if (Math.abs(p) < maxPower) {
            if (p != 0) p *= Math.pow(Math.abs(p) / maxPower, kE - 1);
            integral += error * delta_time;
        } else {
            integral = 0;
        }

        double i = integral * kI;

        return p + i;
    }

    public double getPower(double rawPower, boolean enable_limiting) { // this should be called on every loop!!!!!

        double delta_time = System.currentTimeMillis() - previous_time;
        double currentPosition = positionSup.getAsDouble();
        double power;

        if (enable_limiting) {
            if (currentPosition < minPosition || last_manually_targeted_position <= minPosition) {
                rawPower = Math.max(0, rawPower);
                setTarget(minPosition);
            } else if (currentPosition > maxPosition || last_manually_targeted_position >= maxPosition) {
                rawPower = Math.min(0, rawPower);
                setTarget(maxPosition);
            }
        }

        if (Math.abs(rawPower) > minPower) {
            power = rawPower;
            last_manually_targeted_position = currentPosition;
            target_position = currentPosition;
            integral = 0;
            last_time = System.currentTimeMillis() / 1000.0;
        } else if (System.currentTimeMillis() / 1000.0 - last_time < calibration_time) {
            power = 0;
            target_position = currentPosition;
            integral = 0;
        } else {
            power = getCorrection();
        }

        power = Math.max(Math.min(power, Math.min(maxPower, previous_power + acceleration * delta_time)), Math.max(previous_power - acceleration * delta_time, -maxPower));

        previous_time = System.currentTimeMillis() / 1000.0;
        previous_power = power;
        if (Math.abs(power) < minPower) return 0; // don't set previous_power to 0
        return power;
    }

    public double getPower(double rawPower) {
        return getPower(rawPower, this.enable_limiting);
    }

    public double pidPower(double multiplier) {
        last_time = 0;
        return multiplier * getPower(0, false);
    }

    public double pidPower() {
        return pidPower(1);
    }

    public void brake() {
        setTarget(positionSup.getAsDouble());
    }

    public void disable_limiting() {
        enable_limiting = false;
    }

    public double getCurrentPosition() {
        return positionSup.getAsDouble();
    }
}
