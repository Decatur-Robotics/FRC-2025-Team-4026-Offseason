package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

public class Elevator {
 
    private double position;
    private double voltage;
    private double velocity;

    private ElevatorIO io;

    private MotionMagicVoltage positionRequest;
    private VelocityVoltage velocityRequest;

    public Elevator(ElevatorIO io){
        this.io = io;
        position = ElevatorConstants.STOWED_POSITION;
        

        
    }

    public void periodic(){
        io.updateInputs(null);
        Logger.processInputs(null, null);
         

        
    
    }

    public void setPosition(ElevatorIOTalonFX mainMotor){
        this.position = position;
        mainMotor.mainMotor.setControl(positionRequest.withPosition(position));
    }

    public double getPosition(ElevatorIOTalonFX mainMotor){
        return position;
    }

    public Command zeroCommand(ElevatorIOTalonFX mainMotor){
        return Commands.sequence(
        Commands.runOnce(() -> {
            mainMotor.mainMotor.setVoltage(voltage);
        }),
        Commands.runOnce(() -> {
            mainMotor.mainMotor.setPosition(0.0);
        }));
    }

}
