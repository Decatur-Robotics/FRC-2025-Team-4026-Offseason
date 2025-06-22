package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs{
        public ElevatorIOData data = new ElevatorIOData(false, false, 0.0, 0.0, 0.0, 0.0);
    }
    
    record ElevatorIOData(
        Boolean mainMotorConnected,
        Boolean followerMotorConnected,
        Double position,
        Double voltage,
        Double velocity,
        double supplyAmps
    ){

       
    } 
      
    default void updateInputs(ElevatorIOInputs inputs) {
            
    }

} 
