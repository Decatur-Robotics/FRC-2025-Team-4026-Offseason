package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
public class Arm extends SubsystemBase{
    private double position;
    private double voltage;
    private double velocity;

    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private MotionMagicVoltage positionRequest;
    private ArmIO io;
    private boolean isEstopped;

    public Arm (ArmIO io){
        this.io = io;
        position = ArmConstants.STOWED_POSITION;
    }

    public void periodic(){
        io.updateInputs(null);
        Logger.processInputs(null, null);

        if(isEstopped){
            io.stop();
        }
    }

    public void setPosition(ArmIOTalonFX motor){
        motor.motor.setControl(positionRequest.withPosition(position));
    }

    public double getPosition(ArmIOTalonFX motor){
        return inputs.data.position();
    }

    public Command zeroCommand(ArmIOTalonFX motor){
        return Commands.sequence(
        Commands.runOnce(() -> {
            motor.motor.setVoltage(voltage);
        }),
        Commands.runOnce(() -> {
            motor.motor.setPosition(0.0);
        }));
    }
}
