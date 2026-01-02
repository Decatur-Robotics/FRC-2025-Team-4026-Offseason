package frc.robot.subsystems.superstructure.arm;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOData;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.arm.ArmConstants.ArmHardwareConstants;

public class ArmIOSim implements ArmIO{
    // public static final Double armMassKG = Units.lbsToKilograms(16);
    // public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1).withReduction(45);

    private ArmHardwareConstants hardwareConstants = ArmConstants.HARDWARE_CONSTANTS;
    private final SingleJointedArmSim armSim;
    private final SimulatedMotorController.GenericMotorController motorController;

    private Voltage targetVoltage;
   // private final Angle encoderOffset;
    

    public ArmIOSim(){

        this.armSim = new SingleJointedArmSim(hardwareConstants.ARM_GEARBOX(), hardwareConstants.ARM_GEARING_REDUCTION(), SingleJointedArmSim.estimateMOI(hardwareConstants.ARM_LENGTH().in(Meters), hardwareConstants.ARM_MASS().in(Kilograms)), hardwareConstants.ARM_MASS().in(Kilograms), hardwareConstants.ARM_MIN_ANGLE().in(Degrees), hardwareConstants.ARM_MAX_ANGLE().in(Degrees), true, hardwareConstants.ARM_MAX_ANGLE().in(Degrees));

        this.motorController = new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60Foc(1));
        this.targetVoltage = Volts.zero();

        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        armSim.update(0.0);

    }

    public void updateInputs(ArmIOInputs inputs){
        Angle armAngle = Radians.of(armSim.getAngleRads()*hardwareConstants.ARM_GEARING_REDUCTION());
        AngularVelocity armAngularVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec()*hardwareConstants.ARM_GEARING_REDUCTION());
        Voltage realVoltage = motorController.constrainOutputVoltage(armAngle, armAngularVelocity, targetVoltage);
        realVoltage = SimulatedBattery.clamp(realVoltage);
        if(DriverStation.isDisabled()){
            realVoltage = Volts.zero();
        }
        armSim.setInputVoltage(realVoltage.in(Volts));
        //Robot simulation is iterated 5 times, less for performance, more for accurracy
        for (int i = 0; i < 5; i++) armSim.update(Robot.defaultPeriodSecs / 5);

        inputs.data = new ArmIOData(true , realVoltage.in(Volts), armAngle.in(Radians), armAngularVelocity.in(RadiansPerSecond), Amps.of(armSim.getCurrentDrawAmps()).in(Amps));
    }

    private Current getSupplyCurrent(){
        return Amps.of(armSim.getCurrentDrawAmps());
    }

    public void setVoltage(Voltage voltage) {
        this.targetVoltage = voltage;
    }
}
