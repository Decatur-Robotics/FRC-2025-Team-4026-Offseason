package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
 
    private double position;
    private double voltage;
    private double velocity;

    
    private boolean isEStopped = false;
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private MotionMagicVoltage positionRequest;
    private VelocityVoltage velocityRequest;

    static{
        switch (Constants.getRobotType()) {
            case COMPETITION, OFFSEASON -> {
                
            }
                
                
        
            case SIMULATION -> {

            }
                
        }
    }

    @AutoLogOutput(key = "Elevator/Profile/AtGoal")
    private boolean atGoal = false;

    public Elevator(ElevatorIO io){
        this.io = io;
        position = ElevatorConstants.STOWED_POSITION;
        

        
    }

    public void periodic(){
        io.updateInputs(null);
        Logger.processInputs(null, null);
         

        
    if (isEStopped) {
        io.stop();
      }
    
    }

    public void setPosition(ElevatorIOTalonFX mainMotor){
        this.position = position;
        mainMotor.mainMotor.setControl(positionRequest.withPosition(position));
    }

    public double getPosition(ElevatorIOTalonFX mainMotor){
        return inputs.data.position();
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
