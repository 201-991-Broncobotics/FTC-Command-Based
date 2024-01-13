package org.firstinspires.ftc.teamcode.commands.examplecommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

public class TurnForever extends CommandBase {

    private final DriveSubsystemBase driveTrain;
    private static boolean isRunning = false;

    public TurnForever(DriveSubsystemBase driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        isRunning = true;
    }

    @Override
    public void execute() {
        driveTrain.drive(0, 0, 0.5, 1);
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
    }

    public static boolean isRunning() {
        return isRunning;
    }
}