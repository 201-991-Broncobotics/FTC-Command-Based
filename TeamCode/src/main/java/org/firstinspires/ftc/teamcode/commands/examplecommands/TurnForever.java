package org.firstinspires.ftc.teamcode.commands.examplecommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import java.util.function.BooleanSupplier;

public class TurnForever extends CommandBase {

    private final Mecanum mecanum;
    private static boolean isRunning = false;

    public TurnForever(Mecanum mecanum) {
        this.mecanum = mecanum;
        addRequirements(mecanum);
    }

    @Override
    public void initialize() {
        isRunning = true;
    }

    @Override
    public void execute() {
        mecanum.drive(0, 0, 0.5, false);
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
    }

    public static boolean isRunning() {
        return isRunning;
    }
}