package org.firstinspires.ftc.teamcode.commands.defaultcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import static org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import java.util.function.DoubleSupplier;

public class TeleOpDrive extends CommandBase {

    private final Mecanum mecanum;
    private final DoubleSupplier strafe, forward, turn, slow;

    public TeleOpDrive(Mecanum mecanum, DoubleSupplier strafeSup, DoubleSupplier forwardSup, DoubleSupplier turnSup, DoubleSupplier slowSup) {
        this.mecanum = mecanum;
        addRequirements(mecanum);

        strafe = strafeSup;
        forward = forwardSup;
        turn = turnSup;
        slow = slowSup;
    }

    @Override
    public void initialize() {
        // don't do anything
    }

    @Override
    public void execute() {
        double strafe_val = normalize_joystick(strafe.getAsDouble());
        double forward_val = normalize_joystick(forward.getAsDouble());
        double turn_val = normalize_joystick(turn.getAsDouble());
        double slow_val = normalize_joystick(slow.getAsDouble());
        mecanum.drive(
            strafe_val * slow_val,
            forward_val * slow_val,
            turn_val * slow_val
        );
    }

    @Override
    public void end(boolean interrupted) {
        // don't do anything
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
