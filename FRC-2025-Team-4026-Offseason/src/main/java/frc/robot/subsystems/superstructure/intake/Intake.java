package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private double voltage;
    private double velocity;
    private IntakeIOTalonFX motorLeft;

    private final String inputsName;
    private boolean isEStopped = false;
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private VelocityVoltage velocityRequest;

    static{
        switch(Constants.getRobotType()){
            case COMPETITION, OFFSEASON -> {

            }

            case SIMULATION -> {

            }
        }
    }

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputsName = this.getClass().getSimpleName() + "Inputs";
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);

        if (isEStopped){
            io.stop();
        }
        
    }

    public void setVelocity(double velocity){
        this.velocity = velocity;
        velocityRequest = new VelocityVoltage(velocity);
        motorLeft.motorLeft.setControl(velocityRequest);
    }

    public Command setVelocityCommand(double velocity){
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public double getVelocity(){
        return velocity;
    }


    
}
