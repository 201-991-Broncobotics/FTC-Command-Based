package org.firstinspires.ftc.teamcode.commands.utilcommands;

import com.arcrobotics.ftclib.command.CommandBase;

public class Wait extends CommandBase {
    private final double time;
    private double end_time;

    public Wait(double time) {
        this.time = time;
    }

    @Override
    public void initialize() {
        end_time = System.currentTimeMillis() / 1000.0 + time;
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() / 1000.0 > end_time;
    }
}
