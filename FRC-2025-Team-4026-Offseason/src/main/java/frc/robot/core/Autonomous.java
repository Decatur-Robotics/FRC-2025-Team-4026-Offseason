package frc.robot.core;

import java.nio.file.Path;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import frc.robot.subsystems.drive.Drive.PathLocation;
import frc.robot.RobotContainer;
import frc.robot.constants.PathSetpoints;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class Autonomous {
    private RobotContainer robotContainer;
    private Superstructure superstructure;
    private Drive swerve;

    public Autonomous(RobotContainer robotContainer, Superstructure superstructure, Drive swerve){
        this.robotContainer = robotContainer;
        this.superstructure = superstructure;
        this.swerve = swerve;
        registerNamedCommands();
    }

    public void registerNamedCommands(){
            final Superstructure superstructure = robotContainer.getSuperstructure();
            NamedCommands.registerCommand("L4", superstructure.scoreCoralL4Command(()->true, ()->true, ()->true, ()->true));
            NamedCommands.registerCommand("HP Intake", superstructure.intakeCoralHumanPlayerCommand());
    }

    private Command autoL4Command() {
        return superstructure.scoreCoralL4Command(() -> swerve.isNearAligned(), () -> swerve.isAligned(), 
            () -> (Robot.isSimulation() && swerve.isNearAligned()), 
            () -> (Robot.isSimulation() && swerve.isAligned()));
    }

    public Command getAutoCommand(){
      //  return new PathPlannerAuto("3 Coral Left");
        return
            threeCoralHumanPlayerAuto(PathSetpoints.RED_RIGHT_HUMAN_PLAYER_RIGHT, 
                PathSetpoints.RED_REEF_E, PathSetpoints.RED_REEF_C, PathSetpoints.RED_REEF_D);
    }

        private Command threeCoralHumanPlayerAuto(Pose2d humanPlayerPose, 
            Pose2d firstBranchPose, Pose2d secondBranchPose, Pose2d thirdBranchPose) {
        return Commands.sequence(
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(firstBranchPose, PathLocation.Reef)),
            Commands.deadline(
                superstructure.intakeCoralHumanPlayerCommand(), 
                swerve.driveToHumanPlayerFromReefBacksideAuto(humanPlayerPose, firstBranchPose)),
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(secondBranchPose, PathLocation.Reef)),
            Commands.deadline(
                superstructure.intakeCoralHumanPlayerCommand(), 
                swerve.driveToPoseAuto(humanPlayerPose, PathLocation.HumanPlayer)),
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(thirdBranchPose, PathLocation.Reef)),
            Commands.deadline(
                superstructure.intakeCoralHumanPlayerCommand(), 
                swerve.driveToPoseAuto(humanPlayerPose, PathLocation.HumanPlayer))
        );
    }
}
