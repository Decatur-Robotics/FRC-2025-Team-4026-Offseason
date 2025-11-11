package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private double voltage;
    private double velocity;

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
        
    }

    public void periodic(){
        io.updateInputs(null);
        Logger.processInputs(null, null);

        if (isEStopped){
            io.stop();
        }
        
    }

    public void setVelocity(IntakeIOTalonFX motorRight, IntakeIOTalonFX motorLeft, double velocity){
        this.velocity = velocity;
        velocityRequest = new VelocityVoltage(velocity);
        motorRight.motorRight.setControl(velocityRequest);
        motorLeft.motorLeft.setControl(velocityRequest);
    }

    public double getVelocity(){
        return velocity;
    }


    
}
