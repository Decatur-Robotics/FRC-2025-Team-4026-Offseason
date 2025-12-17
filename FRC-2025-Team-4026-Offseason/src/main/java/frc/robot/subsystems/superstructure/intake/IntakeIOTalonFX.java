package frc.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.Ports;
import frc.robot.subsystems.superstructure.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.superstructure.wrist.WristConstants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class IntakeIOTalonFX implements IntakeIO{
    public TalonFX motorLeft, motorRight;

    private TalonFXConfiguration config = new TalonFXConfiguration();

    private VoltageOut voltageRequest;
    
    private final StatusSignal<Voltage> voltageRight;
    private final StatusSignal<AngularVelocity> velocityRight;
    private final StatusSignal<Voltage> voltageLeft;
    private final StatusSignal<AngularVelocity> velocityLeft;
    private final StatusSignal<Current> currentLeft;
    private final StatusSignal<Current> currentRight;
    private VelocityVoltage velocityRequest;


    public IntakeIOTalonFX(){
        motorRight = new TalonFX(Constants.getRobotType() == RobotType.COMPETITION ? Ports.INTAKE_MOTOR_RIGHT : Ports.INTAKE_MOTOR_RIGHT);
        motorLeft = new TalonFX(Constants.getRobotType() == RobotType.COMPETITION ? Ports.INTAKE_MOTOR_LEFT : Ports.INTAKE_MOTOR_LEFT);

        motorLeft.setControl(new Follower(Ports.INTAKE_MOTOR_RIGHT, true));

        config.Slot0 = new Slot0Configs()
            .withKP(IntakeConstants.kP)
            .withKI(IntakeConstants.kI)
            .withKD(IntakeConstants.kD)
            .withKS(IntakeConstants.kS)
            .withKV(IntakeConstants.kV)
            .withKA(IntakeConstants.kA);
        voltageRight = motorRight.getMotorVoltage();
        velocityRight = motorRight.getVelocity();
        voltageLeft = motorLeft.getMotorVoltage();
        velocityLeft = motorLeft.getVelocity();
        currentLeft = motorLeft.getSupplyCurrent();
        currentRight = motorRight.getSupplyCurrent();
        tryUntilOk(5,() -> BaseStatusSignal.setUpdateFrequencyForAll(40.0,
        voltageRight, velocityRight, voltageLeft, velocityLeft));
        //replace with port
        tryUntilOk(5, () -> motorRight.optimizeBusUtilization());
    
    }

    public void periodic(){
        if (motorRight.hasResetOccurred() || motorLeft.hasResetOccurred()){
            motorRight.optimizeBusUtilization();
            motorLeft.optimizeBusUtilization();
            motorRight.getPosition().setUpdateFrequency(40);
        }
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeData = new IntakeIO.IntakeIOData(
            BaseStatusSignal.isAllGood(
                    voltageLeft, velocityLeft),
                    BaseStatusSignal.isAllGood(
                       voltageRight, velocityRight),
            voltageRight.getValueAsDouble(),
            velocityRight.getValueAsDouble(),
            currentRight.getValueAsDouble(),
            voltageLeft.getValueAsDouble(),
            velocityLeft.getValueAsDouble(),
            currentLeft.getValueAsDouble()
        );
    }

    public void setVoltage(double voltage) {
        voltageRequest = new VoltageOut(voltage);
        motorRight.setControl(voltageRequest);
        motorLeft.setControl(voltageRequest);
    }

        public void setVelocity(double velocity){
        velocityRequest = new VelocityVoltage(velocity);
        motorLeft.setControl(velocityRequest);
    }

    public void stop(){
        motorRight.stopMotor();
        motorLeft.stopMotor();
    }

    public void setPID(IntakeConstants constants){
        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kS = constants.kS;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
    }

}
