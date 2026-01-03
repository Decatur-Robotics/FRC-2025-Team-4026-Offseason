package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class WristIOSim implements WristIO {

  private final DCMotorSim wristSim;
  private final SimulatedMotorController.GenericMotorController motorController;
  private Voltage targetVoltage;

    public WristIOSim() {
      this.wristSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.05,0.01), DCMotor.getKrakenX60(1));
      this.targetVoltage = Volts.zero();
      this.motorController = new SimulatedMotorController.GenericMotorController(DCMotor.getKrakenX60(1));
      SimulatedBattery.addElectricalAppliances(this::getSupplyCurrent);
      wristSim.update(0.0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    Angle wristPosition = Radians.of(wristSim.getAngularPositionRad()/15);
    AngularVelocity wristVelocity = RadiansPerSecond.of(wristSim.getAngularVelocityRadPerSec()/15);
    Voltage realVoltage = motorController.constrainOutputVoltage(wristPosition, wristVelocity, targetVoltage);
    inputs.data = new WristIOData(wristPosition.in(Radians), wristVelocity.in(RadiansPerSecond),realVoltage.in(Volts), getSupplyCurrent().in(Amps),true);
  }

  public Current getSupplyCurrent(){
       return Amps.of(wristSim.getCurrentDrawAmps());
}
}
