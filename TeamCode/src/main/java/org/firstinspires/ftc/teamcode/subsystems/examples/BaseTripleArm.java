package org.firstinspires.ftc.teamcode.subsystems.examples;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BaseTripleArm extends SubsystemBase {
    /**
     * This is an abstract class to help you create double triple. Extend this class and implement
     * the abstract methods. It will automatically be a subsystem. <br />
     * Set the motors to rawpower
     * Note, this assumes the first two are controlled by motors with the third relatively by a servo
     *
     * @param map
     * @param teleOp               are we in teleOp or autonomous mode
     * @param arm_lengths          the arm lengths, in whatever unit you want. Not completely necessary
     * @param min_angles
     * @param max_angles
     * @param min_powers           the minimum power for it to actually send the power. Due to stall voltage
     * @param max_powers
     * @param accelerations
     * @param calibration_times
     * @param p_coefficients
     * @param e_coefficients
     * @param min_difference
     * @param zero_positions       what the arm encoders read when both arms are straight ahead
     * @param ticks_per_revolution how many encoder ticks would equate to a full 360° spin. Note
     *                             this isn't necessarily equal to the ticks per revolution of the
     *                             motor if it has been geared down.
     * @param servo_max_range      the range of the wrist powered by the servo. Note this isn't
     *                             necessarily equal to the range of the servo if it has been geared down
     * @param tolerances
     * @param absolute
     * @see DoubleArm_Example
     */
    public BaseTripleArm(HardwareMap map, boolean teleOp, double[] arm_lengths, double[] min_angles, double[] max_angles,
                         double[] min_powers, double[] max_powers, double[] accelerations, double[] calibration_times, double[] p_coefficients,
                         double[] e_coefficients, double[] min_difference, double[] zero_positions, double[] ticks_per_revolution, double servo_max_range,
                         double[] tolerances, boolean absolute) {
        ServoEx yo = map.get(ServoEx.class, "hi");
        yo.getAngle();
    }

/*
    private final double[] arm_lengths;

    public final double min_difference;
    public final boolean absolute;

    public final PIECalculator firstMotorCalculator, secondMotorCalculator; */

    /**
     * This is an abstract class to help you create double arms. Extend this class and implement
     * the abstract methods. It will automatically be a subsystem. <br />
     * Set the motors to rawpower
     *
     * @param  //teleOp  are we in teleOp or autonomous mode
     * @param  //arm_lengths the arm lengths, in whatever unit you want. Not completely necessary
     * @param  //min_powers the minimum power for it to actually send the power. Due to stall voltage
     * @param  //zero_positions what the arm encoders read when both arms are straight ahead
     * @param  //ticks_per_revolution how many encoder ticks would equate to a full 360° spin. Note
     *                             this isn't necessarily equal to the ticks per revolution of the
     *                             motor if it has been geared down.
     * @see    DoubleArm_Example
     */
    /*
    public void BaseDoubleArm(HardwareMap map, boolean teleOp, double[] arm_lengths, double[] min_angles, double[] max_angles,
                         double[] min_powers, double[] max_powers, double[] accelerations, double[] calibration_times,
                         double[] p_coefficients, double[] e_coefficients, double min_difference,
                         double[] zero_positions, double[] ticks_per_revolution, double[] tolerances, boolean absolute) {
        init(map, teleOp); // for example, you don't want to reset encoders in teleOp but you do in autonomous

        this.arm_lengths = arm_lengths;

        this.min_difference = min_difference;

        this.absolute = absolute;

        firstMotorCalculator = new PIECalculator(
                p_coefficients[0], e_coefficients[0], min_angles[0], max_angles[0], min_powers[0], max_powers[0],
                accelerations[0], calibration_times[0], tolerances[0], () -> (get_encoder_readings()[0] - zero_positions[0]) / ticks_per_revolution[0] * 360.0
        );

        secondMotorCalculator = new PIECalculator(
                p_coefficients[1], e_coefficients[1], min_angles[1], max_angles[1], min_powers[1], max_powers[1],
                accelerations[1], calibration_times[1], tolerances[1], () -> (get_encoder_readings()[1] - zero_positions[1]) / ticks_per_revolution[1] * 360.0
        );
    }

    public void setTargetAngles(double[] target_angles) {
        firstMotorCalculator.setTarget(target_angles[0]);
        secondMotorCalculator.setTarget(target_angles[1]);
    }

    public double[] getTargetAngles() {
        return new double[] {
                firstMotorCalculator.getTarget(),
                secondMotorCalculator.getTarget()
        };
    }

    public void powerArm(double first_motor_raw_power, double second_motor_raw_power) {
        power_motors(firstMotorCalculator.getPower(first_motor_raw_power), secondMotorCalculator.getPower(second_motor_raw_power));
    }

    public void pidPowerArm(double[] multipliers) { // goes toward target without regard for if it fits limiting
        power_motors(firstMotorCalculator.pidPower(multipliers[0]), secondMotorCalculator.pidPower(multipliers[1]));
    }

    public void pidPowerArm() {
        pidPowerArm(new double[] {1, 1});
    }

    public void pidPowerProximalMaxDistal(double[] multipliers) { // powers proximal, while keeping distal as high as possible at all times
        secondMotorCalculator.setTarget(Math.min(Math.min(90, secondMotorCalculator.getMaxPosition()), firstMotorCalculator.getCurrentPosition()) + 180 - min_difference);
        pidPowerArm(multipliers);
    }

    public void pidPowerProximalMaxDistal() { // powers proximal, while keeping distal as high as possible at all times
        secondMotorCalculator.setTarget(Math.min(Math.min(90, secondMotorCalculator.getMaxPosition()), firstMotorCalculator.getCurrentPosition()) + 180 - min_difference);
        pidPowerArm(new double[] {1, 1});
    } */

    public double[] angles_to_position(double[] angles) {
        return new double[2];
    }

    public double[] position_to_angles(double[] position, boolean concave_up) {
        return new double[2];
    }

    public double[] position_to_angles(double[] position) {
        return position_to_angles(position, true);
    }

    // The code below has to be redefined by the user because it varies between different double arms

    /** Initialize motors, variables, etc. Can also be initialized in the constructor */
    protected abstract void init(HardwareMap map, boolean teleOp);
    /** Get the raw encoder values from the two arms. Increasing should be up. If increasing goes down, introduce a multiplier */
    protected abstract double[] get_encoder_readings();
    /** Put power to the motors. Positive should go up and negative should go down */
    protected abstract void power_motors(double first_power, double second_power);
}
