package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator {
 
    private double position;
    private double voltage;
    private double velocity;

    private MotionMagicVoltage positionRequest;
    private VelocityVoltage velocityRequest;

    public Elevator(ElevatorIO io){
        //this.io = io;
        

        
    }

}
