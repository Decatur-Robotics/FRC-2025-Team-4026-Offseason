package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristIOTalonFX extends SubsystemBase {
    public TalonFX motor;
    public double current;
    public TorqueCurrentFOC controlRequest;

    public WristIOTalonFX(){

    motor = new TalonFX(5);
    //replace with a port

    motor.getConfigurator().apply(new TalonFXConfiguration());
    // should be in constants with more things

    motor.optimizeBusUtilization();
    motor.getVelocity().setUpdateFrequency(20);
    motor.getRotorVelocity().setUpdateFrequency(20);
    motor.getStatorCurrent().setUpdateFrequency(20);
    //current = WristConstants.PERPENDICULAR_CURRENT;

    controlRequest = new TorqueCurrentFOC(current);
    motor.setControl(controlRequest.withOutput(current));
    }
    
}
