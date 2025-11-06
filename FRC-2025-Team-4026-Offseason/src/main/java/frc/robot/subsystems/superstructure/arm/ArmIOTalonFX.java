package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Ports;

public class ArmIOTalonFX {
    public TalonFX motor; 
    public TalonFXConfiguration config = new TalonFXConfiguration();

    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest;

    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> supplyAmps;
    private final StatusSignal<Current> torqueCurrent;

    public ArmIOTalonFX(){
        motor = new TalonFX(Ports.ARM_MOTOR);
        config.Slot0 = new Slot0Configs()
        .withKA(ArmConstants.kA)
        .withKD(ArmConstants.kD)
        .withKG(ArmConstants.kG)
        .withKI(ArmConstants.kI)
        .withKP(ArmConstants.kP)
        .withKS(ArmConstants.kS)
        .withKV(ArmConstants.kV)
        ;

        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        velocity = motor.getVelocity();
        supplyAmps = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();

        positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(20,position,voltage,velocity,supplyAmps,torqueCurrent);
    }
}
