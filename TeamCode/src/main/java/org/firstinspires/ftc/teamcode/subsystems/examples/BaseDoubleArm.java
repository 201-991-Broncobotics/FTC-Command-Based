package org.firstinspires.ftc.teamcode.subsystems.examples;

import static org.firstinspires.ftc.teamcode.Constants.wheel_names;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BaseDoubleArm extends SubsystemBase { // extend this class

    private final double[] arm_lengths;

    public final double min_difference;
    public final boolean absolute;

    public final PIECalculator firstMotorCalculator, secondMotorCalculator;

    /**
     * This is an abstract class to help you create double arms. Extend this class and implement
     * the abstract methods. It will automatically be a subsystem. <br />
     * Set the motors to rawpower
     *
     * @param  teleOp  are we in teleOp or autonomous mode
     * @param  arm_lengths the arm lengths, in whatever unit you want. Not completely necessary
     * @param  min_powers the minimum power for it to actually send the power. Due to stall voltage
     * @param  zero_positions what the arm encoders read when both arms are straight ahead
     * @param  ticks_per_revolution how many encoder ticks would equate to a full 360° spin. Note
     *                      this isn't necessarily equal to the encoder ticks per revolution of the
     *                      motor if it has been geared down.
     * @see    DoubleArm_Example
     */
    public BaseDoubleArm(HardwareMap map, boolean teleOp, double[] arm_lengths, double[] min_angles, double[] max_angles,
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
    }

    public double[] angles_to_position(double[] angles) {
        return new double[2];
    }

    public double[] position_to_angles(double[] position, boolean concave_up) {
        return new double[2];
    }

    public double[] position_to_angles(double[] position) {
        return position_to_angles(position, true);
    }

    @Override
    public void periodic() { // put data to telemetry

    }

    // The code below has to be redefined by the user because it varies between different double arms

    /** Initialize motors, variables, etc. Can also be initialized in the constructor */
    protected abstract void init(HardwareMap map, boolean teleOp);
    /** Get the raw encoder values from the two arms. Increasing should be up. If increasing goes down, introduce a multiplier */
    protected abstract double[] get_encoder_readings();
    /** Put power to the motors. Positive should go up and negative should go down */
    protected abstract void power_motors(double first_power, double second_power);
}

class DoubleArm_Example extends BaseDoubleArm {
    private MotorEx jointOneLeftMotor, jointOneRightMotor, jointTwoMotor;

    /* Constants */
    private static final double[]
            arm_lengths = {1, 1},
            min_angles = {-90, -90},
            max_angles = {90, 90},
            min_powers = {0.1, 0.1},
            max_powers = {1, 1},
            accelerations = {2.0, 2.0},
            calibration_times = {0.7, 0.7},
            p_coefficients = {0.1, 0.1},
            e_coefficients = {1.5, 1.5},
            zero_positions = {410, -817},
            ticks_per_revolution = {2786.2109868741, 2786.2109868741},
            tolerances = {2, 2};

    private static final double min_difference = 15;
    private static final boolean absolute = true;

    @Override
    protected void init(HardwareMap map, boolean teleOp) {
        jointOneLeftMotor = new MotorEx(map, "JointOneLeftMotor", Motor.GoBILDA.RPM_60);
        jointOneRightMotor = new MotorEx(map, "JointOneRightMotor", Motor.GoBILDA.RPM_60);
        jointTwoMotor = new MotorEx(map, "JointTwoMotor", Motor.GoBILDA.RPM_60);
        jointOneLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        jointOneRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        jointTwoMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        jointOneLeftMotor.setInverted(true);
        jointOneLeftMotor.setRunMode(Motor.RunMode.RawPower);
        jointOneRightMotor.setRunMode(Motor.RunMode.RawPower);
        jointTwoMotor.setRunMode(Motor.RunMode.RawPower);
        if (!teleOp) {
            jointOneLeftMotor.stopAndResetEncoder();
            jointOneRightMotor.stopAndResetEncoder();
            jointTwoMotor.stopAndResetEncoder();
        }
    }

    @Override
    public double[] get_encoder_readings() {
        return new double[] {
            jointOneLeftMotor.encoder.getPosition(),
            jointOneRightMotor.encoder.getPosition()
        };
    }

    @Override
    public void power_motors(double first_power, double second_power) {
        jointOneLeftMotor.set(first_power);
        jointOneRightMotor.set(first_power);
        jointTwoMotor.set(second_power);
    }

    public DoubleArm_Example(HardwareMap map, boolean teleOp) {
        super(map, teleOp, arm_lengths, min_angles, max_angles, min_powers, max_powers,
                accelerations, calibration_times, p_coefficients, e_coefficients, min_difference,
                zero_positions, ticks_per_revolution, tolerances, absolute);
    }
}
