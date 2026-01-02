// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSim;
  private final SimulatedMotorController.GenericMotorController driveMotor, steerMotor;
  private final PIDController driveController;
  private final PIDController steerController;
  private boolean driveClosedLoopActivated = false;
  private boolean steerClosedLoopActivated = false;
  private double targetVelocity = 0.0;
  private Rotation2d desiredSteerFacing = new Rotation2d();

  public ModuleIOSim(SwerveModuleSimulation simulation) {
    this.moduleSim = simulation;
    this.driveMotor = moduleSim.useGenericMotorControllerForDrive();
    this.steerMotor = moduleSim.useGenericControllerForSteer();

    this.driveController = new PIDController(.1, 0, 0);
    this.steerController = new PIDController(100, 0, 0);

    steerController.enableContinuousInput(-Math.PI, Math.PI);
    SimulatedArena.getInstance().addCustomSimulation(subTickNum -> runControlLoops());
    
  }

  public void updateInputs(ModuleIOInputs inputs){
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = moduleSim.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = moduleSim.getDriveMotorStatorCurrent().in(Amps);

    inputs.turnConnected = true;
    inputs.turnAbsolutePosition = moduleSim.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = moduleSim.getSteerMotorAppliedVoltage().in(Volts);
    inputs.turnCurrentAmps = moduleSim.getSteerMotorStatorCurrent().in(Amps);

    inputs.odometryDrivePositionsRad = Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions()).mapToDouble(Angle -> Angle.in(Radians)).toArray();
    inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
  }

  public void runControlLoops() {
    if (driveClosedLoopActivated) caclulateDriveControlLoops();
    else driveController.reset();
    if (steerClosedLoopActivated) caclulateSteerControlLoops();
    else steerController.reset();
  }

  private void caclulateDriveControlLoops(){
    DCMotor driveMotorModel = moduleSim.config.driveMotorConfigs.motor;
    double frictionTorque = driveMotorModel.getTorque(driveMotorModel.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage))*Math.signum(targetVelocity);
    double velocityFeedForward = driveMotorModel.getVoltage(frictionTorque, targetVelocity);
    double feedBackVoltage = driveController.calculate(moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond), targetVelocity/TunerConstants.kDriveGearRatio);

    double driveVoltage = velocityFeedForward+feedBackVoltage;
    driveMotor.requestVoltage(Volts.of(DriverStation.isEnabled() ? driveVoltage : 0.0));
  }

  private void caclulateSteerControlLoops(){
    double steerVoltage = steerController.calculate(moduleSim.getSteerAbsoluteFacing().getRadians(), desiredSteerFacing.getRadians());
    steerMotor.requestVoltage(Volts.of(DriverStation.isEnabled() ? steerVoltage : 0.0));
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoopActivated = false;
    driveMotor.requestVoltage(Volts.of(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    steerClosedLoopActivated = false;
    steerMotor.requestVoltage(Volts.of(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoopActivated = true;
    this.targetVelocity = velocityRadPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    steerClosedLoopActivated = true;
    this.desiredSteerFacing = rotation;
  }
}
