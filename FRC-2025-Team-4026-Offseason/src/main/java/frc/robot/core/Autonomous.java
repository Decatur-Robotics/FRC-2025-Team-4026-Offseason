package frc.robot.core;

import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autonomous {
    private RobotContainer robotContainer;

    public Autonomous(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        registerNamedCommands();
    }

    public void registerNamedCommands(){
            final Superstructure superstructure = robotContainer.getSuperstructure();
            NamedCommands.registerCommand("L4", superstructure.scoreCoralL4Command(()->true, ()->true, ()->true, ()->true));
            NamedCommands.registerCommand("HP Intake", superstructure.intakeCoralHumanPlayerCommand());
    }

    public Command getAutoCommand(){
        return new PathPlannerAuto("3 Coral Left");
    }
}
