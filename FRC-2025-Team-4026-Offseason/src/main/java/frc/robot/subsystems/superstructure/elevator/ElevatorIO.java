package frc.robot.subsystems.superstructure.elevator;


import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface ElevatorIO {

    class ElevatorIOInputs{
        public ElevatorIOData data = new ElevatorIOData(false, false, 0.0, 0.0, 0.0, 0.0 , 0.0);
    }
    
    record ElevatorIOData(
        Boolean mainMotorConnected,
        Boolean followerMotorConnected,
        Double position,
        Double voltage,
        Double velocity,
        double supplyAmps,
        double torqueCurrent
    ){

       
    } 
      
    default void updateInputs(ElevatorIOInputs inputs) {
    }

    default void setVoltage(double voltage) {
    }

   default void stop(){}

   default void runPosition(double position, double feedForward) {
   }

    default void setPID(ElevatorConstants constants) {
    }

} 
