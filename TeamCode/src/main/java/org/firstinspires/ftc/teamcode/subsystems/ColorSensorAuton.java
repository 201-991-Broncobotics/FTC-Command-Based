package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.CSensorBase;
import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

public class ColorSensorAuton extends SubsystemBase { //WIP
    CSensorBase CSensor = new CSensorBase();

    public void InitCSensor() {
        new InstantCommand(() -> CSensor.CSensorStartUp());
        }
    public void movePixelRED() {
        new InstantCommand(() -> CSensor.GetTeamPropDistanceRED());
      //So basically now you have to turn the swerve module a little bit to the right/left of the team prop so that the pixel is in the right place
        //How am I going to do this??????????



     }
    }

