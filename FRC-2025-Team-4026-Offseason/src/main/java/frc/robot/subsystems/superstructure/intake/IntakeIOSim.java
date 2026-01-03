package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.annotation.Target;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.intake.IntakeConstants.IntakeHardwareConstants;


public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSim;
    private Voltage targetVoltage;
    private static IntakeHardwareConstants hardwareConstants = IntakeConstants.HARDWARE_CONSTANTS;
    private final SimulatedMotorController.GenericMotorController motorController;
    private final DCMotorSim leftMotorSim, rightMotorSim;
    public IntakeIOSim(AbstractDriveTrainSimulation driveSim){
        this.intakeSim = IntakeSimulation.OverTheBumperIntake("Coral", driveSim,
        hardwareConstants.INTAKE_WIDTH(), hardwareConstants.INTAKE_MAX_EXTENSION(), IntakeSimulation.IntakeSide.FRONT, 1
        );
        this.targetVoltage = Volts.zero();
        this.leftMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.1, 0.01), hardwareConstants.INTAKE_GEARBOX());
        this.rightMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.1, 0.01), hardwareConstants.INTAKE_GEARBOX());
        this.motorController = new SimulatedMotorController.GenericMotorController(hardwareConstants.INTAKE_GEARBOX());
        SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
    }

    public void setVoltage(Voltage voltage){
        this.targetVoltage = voltage;
        if(targetVoltage.in(Volts) > 0){
            intakeSim.startIntake();
        }
        else{
            intakeSim.stopIntake();
        }
    }

    public void updateInputs(IntakeIOInputs inputs){
        Angle intakeAngle = Radians.of(leftMotorSim.getAngularPositionRad());
        AngularVelocity intakeAngularVelocity = RadiansPerSecond.of(leftMotorSim.getAngularVelocityRadPerSec());
        Voltage realVoltage = motorController.constrainOutputVoltage(intakeAngle, intakeAngularVelocity, targetVoltage);
        realVoltage = SimulatedBattery.clamp(realVoltage);
        inputs.intakeData = new IntakeIOData(true, true, realVoltage.in(Volts), intakeAngularVelocity.in(RadiansPerSecond), realVoltage.in(Volts), intakeAngularVelocity.in(RadiansPerSecond));
    }

    public Current getSupplyCurrent(){
         return Amps.of(leftMotorSim.getCurrentDrawAmps() + rightMotorSim.getCurrentDrawAmps());
     }

    public void setVoltage(double voltage){
        this.targetVoltage = Volts.of(voltage);
    }
}
