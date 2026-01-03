package frc.robot.subsystems.superstructure.elevator;
//hi
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;


import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorHardwareConstants hardwareConstants = ElevatorConstants.HARDWARE_CONSTANTS;

    private final ElevatorSim elevatorSim;
    private final SimulatedMotorController.GenericMotorController motorController;

    private Voltage targetVoltage;
    private double drumCircumference;
    private final PIDController controller = new PIDController(0.0, 0.0, 0.0);

    public ElevatorIOSim() {
        this.drumCircumference = hardwareConstants.ELEVATOR_DRUM_WHEEL_TEETH()*hardwareConstants.CHAIN_LENGTH().in(Meters);
        this.elevatorSim = new ElevatorSim(hardwareConstants.ELEVATOR_GEARBOX(), hardwareConstants.ELEVATOR_GEARING_REDUCTION(), hardwareConstants.ELEVATOR_CARRIAGE_WEIGHT().in(Kilograms), drumCircumference/(2*Math.PI), 0, hardwareConstants.ELEVATOR_MAX_HEIGHT().in(Meters), true, 0);

        this.motorController = new SimulatedMotorController.GenericMotorController(hardwareConstants.ELEVATOR_GEARBOX());
        motorController.withCurrentLimit(ElevatorConstants.STATOR_CURRENT_LIMIT);
         SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
        elevatorSim.update(0.0);
        this.targetVoltage = Volts.zero();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double drumRotations = elevatorSim.getPositionMeters() / hardwareConstants.ELEVATOR_STAGES()/hardwareConstants.CHAIN_LENGTH().in(Meters)/hardwareConstants.ELEVATOR_DRUM_WHEEL_TEETH();
        Angle motorAngle = Rotations.of(drumRotations * hardwareConstants.ELEVATOR_GEARING_REDUCTION());
        double drumVelocityRotationsPerSecond = elevatorSim.getVelocityMetersPerSecond()
        / hardwareConstants.ELEVATOR_STAGES()
        / hardwareConstants.ELEVATOR_DRUM_WHEEL_TEETH()
        / hardwareConstants.CHAIN_LENGTH().in(Meters);
        AngularVelocity motorVelocity = RotationsPerSecond.of(drumVelocityRotationsPerSecond * hardwareConstants.ELEVATOR_GEARING_REDUCTION());
        Voltage realVoltage = motorController.constrainOutputVoltage(motorAngle, motorVelocity, targetVoltage);
        realVoltage = SimulatedBattery.clamp(realVoltage);
        elevatorSim.setInputVoltage(realVoltage.in(Volts));
        //Runs simulation in 5 iterations
        for (int i = 0; i < 5; i++) elevatorSim.update(Robot.defaultPeriodSecs / 5.0);

        inputs.data = new ElevatorIOData(true,
         true, 
         motorAngle.in(Rotations), 
         realVoltage.in(Volts), 
         motorVelocity.in(RotationsPerSecond), 
         getSupplyCurrent().in(Amps), realVoltage.in(Volts), 
         motorVelocity.in(RotationsPerSecond), 
         getSupplyCurrent().in(Amps));
    }


    private Current getSupplyCurrent() {
        return Amps.of(elevatorSim.getCurrentDrawAmps());
    }

    public void setVoltage(double voltage) {
        this.targetVoltage = Volts.of(voltage);
    }




}
    


