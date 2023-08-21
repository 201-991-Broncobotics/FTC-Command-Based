package org.firstinspires.ftc.teamcode.subsystems.untested;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BaseTripleArm extends SubsystemBase {

    /*
    This is an abstract class to help you create triple arm. Extend this class and implement
     * the abstract methods. It will automatically be a subsystem. <br />
     * Set the motors to rawpower
     * Note, this assumes the first two are controlled by motors with the third relatively by a servo
     */


    public BaseTripleArm(HardwareMap map, double[] arm_lengths, double min_y, double max_y,
                         double min_x, double min_separation) {
    }

    public double[] get_absolute_angles() {
        return new double[] {};
    }

    /**
     * @param angles in degrees of the arm, either absolute or relative depending on what they were
     *               set to initially
     * @return the x-y coordinates of the end of the third arm, assuming the joint to the first arm
     *              is the origin, in whatever units used to define the arm lengths
     */
    public double[] angles_to_position(double[] angles) {
        return new double[] {
            // arm_lengths[0] *
        };
    }

    /**
     * @param position target x-y coordinates of the end of the third arm
     * @param target_angle target absolute angle of the third arm
     * @param concave_up whether you want the joint between the first and second arms to be concave
     *                   up or down. Generally if y > 0 you would want concave down and vice versa
     * @return the angles the three joints should be, in whatever units they were set to initially
     */
    public double[] position_to_angles(double[] position, double target_angle, boolean concave_up) {
        return new double[2];
    }

    // The code below has to be redefined by the user because it varies between different double arms

    /** Initialize motors, variables, etc. Can also be initialized in the constructor */
    protected abstract void init(HardwareMap map, boolean teleOp);
    /** Get the raw encoder values from the two arms. Increasing should be up. If increasing goes down, introduce a multiplier */
    protected abstract double[] get_encoder_readings();
    /** Put power to the motors. Positive should go up and negative should go down */
    protected abstract void power_motors(double first_power, double second_power);
}
