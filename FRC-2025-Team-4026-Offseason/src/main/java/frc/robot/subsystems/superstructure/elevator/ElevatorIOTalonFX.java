package frc.robot.subsystems.superstructure.elevator;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import frc.robot.Ports;
import frc.robot.Constants.RobotType;

public class ElevatorIOTalonFX {
    private TalonFX mainMotor, followerMotor;

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> supplyAmps;
    
    


    public ElevatorIOTalonFX(){
        mainMotor = new TalonFX(Constants.getRobotType() == RobotType.COMPETITION ? Ports.ELEVATOR_MOTOR_MAIN : Ports.ELEVATOR_MOTOR_MAIN);
        followerMotor = new TalonFX(Constants.getRobotType() == RobotType.COMPETITION ? Ports.ELEVATOR_MOTOR_FOLLOWER : Ports.ELEVATOR_MOTOR_FOLLOWER);
        followerMotor.setControl(new Follower(Ports.ELEVATOR_MOTOR_MAIN, true));

        config.Slot0 = new Slot0Configs()
            .withKP(ElevatorConstants.kP)
            .withKI(ElevatorConstants.kI)
            .withKD(ElevatorConstants.kD)
            .withKS(ElevatorConstants.kS)
            .withKV(ElevatorConstants.kV)
            .withKA(ElevatorConstants.kA);

        position = mainMotor.getPosition();
        voltage = mainMotor.getMotorVoltage();
        velocity = mainMotor.getVelocity();
        supplyAmps = mainMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(20, position, voltage, velocity, supplyAmps);

        
    }
}
