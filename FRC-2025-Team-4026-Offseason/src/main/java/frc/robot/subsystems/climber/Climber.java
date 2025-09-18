package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;

public class Climber {
    private double position;
    private double voltage;
    private double velocity;

    private ClimberIO io;

    private MotionMagicVoltage positionRequest;
    private VelocityVoltage velocityRequest;

    public Climber(ClimberIO io) {
        this.io = io;
        position = ClimberConstants.STOWED_POSITION;
    }
    
    public void periodic(){
        io.updateInputs(null);
        Logger.processInputs(null, null);

    }

    public void setPosition(ClimberIOTalonFX climbMotor){
        this.position = position;
        climbMotor.climbMotor.setControl(positionRequest.withPosition(position));
    }

    public double getPosition(ClimberIOTalonFX climbMotor){
        return position;
    }

    public Command zeroCommand(ClimberIOTalonFX climbMotor) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                climbMotor.climbMotor.setVoltage(voltage);
            }),
            Commands.runOnce(() -> {
                climbMotor.climbMotor.setPosition(0.0);
            }));
    }


}
