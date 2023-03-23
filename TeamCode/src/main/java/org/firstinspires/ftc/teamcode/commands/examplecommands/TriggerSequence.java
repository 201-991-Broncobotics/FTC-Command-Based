package org.firstinspires.ftc.teamcode.commands.examplecommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class TriggerSequence extends SequentialCommandGroup {

    public boolean isInactive = true, success = true;
    public int trigger_number = 0;
    private final int number_of_triggers;

    public TriggerSequence(double consecutive_time, BooleanSupplier ... triggers) {

        number_of_triggers = triggers.length;
        if (number_of_triggers < 2) throw new IllegalArgumentException("Must have at least one condition in a trigger sequence");
        reset(); // only in constructor

        new Trigger(triggers[0]).and(new Trigger(() -> isInactive)).whenActive(this); // I think this works? Might have to be at end of constructor though

        addCommands(new InstantCommand(this::init)); // initialization command

        for (int i = 1; i < triggers.length; i++) { // start at one because we start only when the first condition is hit
            addCommands(
                new CheckTrigger(triggers[i], consecutive_time, this),
                new ConditionalCommand(new InstantCommand(() -> trigger_number += 1), new InstantCommand(this::finish), () -> success)
            );
        }
        addCommands(new InstantCommand(this::finish));

        /* LOGIC
        Command starts once the first trigger is activated and its currently not active
        Then we do the initialization function
        Then we loop:
            check the next trigger
                this.success updates based on whether it was interrupted or not
                if it worked, then continue
                if it failed, then mark it as a fail
            once we're done, if we haven't failed, we've succeeded */

    }

    public void reset() {
        trigger_number = 0;
        isInactive = true;
    }

    private void init() {
        trigger_number = 1;
        isInactive = false;
    }

    private void finish() {
        isInactive = true; // don't reset trigger_number
        cancel();
    }

    public Command triggerSequenceCommand(Command ... commands) {
        return new SequentialCommandGroup(new InstantCommand(this::reset), new SequentialCommandGroup(commands));
    }

    public Trigger getEndTrigger() {
        return slice(number_of_triggers);
    }

    public Trigger slice(int index) {
        return new Trigger(() -> (trigger_number == index) && (isInactive)); // it's good IFF
    }
}

class CheckTrigger extends CommandBase {

    private final BooleanSupplier condition;
    private final TriggerSequence triggerSequence;
    private final double time;
    private double end_time;

    public CheckTrigger(BooleanSupplier condition, double time, TriggerSequence triggerSequence) {
        this.condition = condition;
        this.time = time;
        this.triggerSequence = triggerSequence;
    }

    @Override
    public void initialize() {
        end_time = System.currentTimeMillis() / 1000.0 + time;
    }

    @Override
    public void end(boolean interrupted) {
        triggerSequence.success = condition.getAsBoolean();
    }

    @Override
    public boolean isFinished() {
        return (condition.getAsBoolean()) || (System.currentTimeMillis() / 1000.0 > end_time);
    }
}