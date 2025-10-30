package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs{
        public ArmIOData data = new ArmIOData(false,0.0,0.0,0.0,0.0,0.0);
    }
    record ArmIOData(
    boolean motorConnected,
    double voltage,
    double position,
    double velocity,
    double supplyAmps,
    double torqueCurrent
    ){}; 
    
} 
