// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive.PathLocation;
import frc.robot.subsystems.vision.VisionConsumer;
import frc.robot.util.LocalADStarAK;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements VisionConsumer{
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;

  private PIDController translationalController = new PIDController(
        5.75, 0, 0.2);
        // 5.25, 0, 0.3); 
    private PIDController rotationalController = new PIDController(
        6.25, 0, 0.3);

  private RobotConfig config; // SwerveConstants.CONFIG;
  private SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
    config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
    Units.rotationsToRadians(10.76) );

      private SwerveSetpoint previousSetpoint;

      private SwerveSetpointGenerator swerveSetpointGenerator = new SwerveSetpointGenerator(
        config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
        Units.rotationsToRadians(10.76));
  
  private Pose2d targetPose = null;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default().withRobotMass(Kilograms.of(ROBOT_MASS_KG))
  .withCustomModuleTranslations(getModuleTranslations()).withGyro(COTS.ofPigeon2()).withSwerveModule(
    new SwerveModuleSimulationConfig(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1),
     TunerConstants.FrontLeft.DriveMotorGearRatio, TunerConstants.FrontLeft.SteerMotorGearRatio, 
    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage), Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage), 
    Meters.of(TunerConstants.FrontLeft.WheelRadius), KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia), WHEEL_COF));


    public enum PathLocation {
      None(List.of()),
      Reef(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)),
      Processor(List.of(3, 16)),
      Net(List.of(4, 5, 14, 15)),
      HumanPlayer(List.of(1, 2, 12, 13));

      private final List<Integer> apriltagIds;

      private PathLocation(List<Integer> apriltagIds) {
          this.apriltagIds = apriltagIds;
      }

      public List<Integer> getApriltagIds() {
          return this.apriltagIds;
      }
  }

  public boolean isAligned() {
    boolean velocityAligned = true; 

    for (SwerveModuleState module : getModuleStates()) {
        if (module.speedMetersPerSecond > 0.1) 
            velocityAligned = false;
    }

    return isAtTargetPose() && velocityAligned;
}

public boolean isAtTargetPose() {
  if (targetPose == null) return false;
  
  boolean isAtTargetX = Math.abs(translationalController.getError()) < 0.025;
  boolean isAtTargetRotation = Math.abs(rotationalController.getError()) < 0.03;

  return isAtTargetX && isAtTargetRotation;
}

  private PathLocation targetPoseLocation = PathLocation.None;

  public boolean isNearAligned() {
    boolean velocityNearAligned = true;

    for (SwerveModuleState module : getModuleStates()) {
        if (module.speedMetersPerSecond > 1.5) 
            velocityNearAligned = false;
    }

    return isNearTargetPose() && velocityNearAligned;
}
public boolean isNearTargetPose() {
  if (targetPose == null) return false;
  
  boolean isNearTargetX = Math.abs(translationalController.getError()) < .2;
  boolean isNearTargetRotation = Math.abs(rotationalController.getError()) < .075;

  return isNearTargetX && isNearTargetRotation;
}


  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    configureAutoBuilder();
    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    //PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     this::getChassisSpeeds,
    //     this::runVelocity,
    //     new PPHolonomicDriveController(
    //         new PIDConstants(.1, 0.0, 0.0), new PIDConstants(100.0, 0.0, 1.5)),
    //     PP_CONFIG,
    //     () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    //     this);
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  private void configureAutoBuilder() {
        try {
          RobotConfig config = RobotConfig.fromGUISettings(); // SwerveConstants.CONFIG;
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::setPose,         // Consumer for seeding pose against auto
                this::getChassisSpeeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> driveAuto(() -> speeds, () -> feedforwards),
                // setControl(
                //     driveRequest.withSpeeds(speeds)
                //         .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                //         .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                // ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(.1, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(100, 0, 1)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );

            setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(10.76) 
        );
        ChassisSpeeds currentSpeeds = getChassisSpeeds(); 
        SwerveModuleState[] currentStates = getModuleStates(); // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));

        } 
        catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }


  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      Logger.recordOutput("Robot Pose", getPose());

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);


    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return new Pose2d(poseEstimator.getEstimatedPosition().getMeasureX(), poseEstimator.getEstimatedPosition().getMeasureY(), poseEstimator.getEstimatedPosition().getRotation());
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  @Override
  public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

   public Command driveToPoseAuto(Pose2d targetPose,
            PathLocation targetPoseLocation) {
        return Commands.run(() -> driveToPose(() -> new ChassisSpeeds(0, 0, 0), 
                    () -> targetPose, targetPoseLocation, 4), this)
            .finallyDo(() -> {
                this.targetPose = null;
                this.targetPoseLocation = PathLocation.None;
            });
    }
  
    public void driveToPose(Supplier<ChassisSpeeds> speeds, Supplier<Pose2d> targetPose,
            PathLocation targetPoseLocation, double maxSpeed) {
        this.targetPose = targetPose.get();
        this.targetPoseLocation = targetPoseLocation;

        double targetRotation = speeds.get().omegaRadiansPerSecond;

        if (speeds.get().omegaRadiansPerSecond == 0) {
            targetRotation = rotationalController.calculate(
                getPose().getRotation().getRadians(), this.targetPose.getRotation().getRadians());
        }

        if (speeds.get().vxMetersPerSecond == 0 && speeds.get().vyMetersPerSecond == 0) {
            double targetTranslation = translationalController.calculate(
                0, getPose().getTranslation().getDistance(this.targetPose.getTranslation()));

            // if (targetTranslation > maxSpeed) targetTranslation = maxSpeed;

            ChassisSpeeds newSpeeds = new ChassisSpeeds(targetTranslation, 
                0, targetRotation);

            Rotation2d travelRotation = this.targetPose.getTranslation().minus(getPose().getTranslation()).getAngle();

            this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                newSpeeds, getPose().getRotation().minus(travelRotation)));
        }
        else {
            ChassisSpeeds newSpeeds = new ChassisSpeeds(speeds.get().vxMetersPerSecond, speeds.get().vyMetersPerSecond, targetRotation);

            this.driveRobotRelative(newSpeeds);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
      // Note: it is important to not discretize speeds before or after
      // using the setpoint generator, as it will discretize them for you
      previousSetpoint = setpointGenerator.generateSetpoint(
          previousSetpoint, // The previous setpoint
          speeds, // The desired target speeds
          0.02 // The loop time of the robot code, in seconds
      );}
      public Command driveAuto(Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) {
        return run(() -> {
            // // Note: it is important to not discretize speeds before or after
            // // using the setpoint generator, as it will discretize them for you
            // previousSetpoint = setpointGenerator.generateSetpoint(
            //     previousSetpoint, // The previous setpoint
            //     speeds.get(), // The desired target speeds
            //     0.02 // The loop time of the robot code, in seconds
            // );

            // setControl(driveRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds())
            //     .withWheelForceFeedforwardsX(feedforwards.get().robotRelativeForcesXNewtons())
            //     .withWheelForceFeedforwardsY(feedforwards.get().robotRelativeForcesYNewtons()));

            System.out.println("driving auto"); 

            driveRobotRelative(speeds.get());
        });
    }

     public Command driveToHumanPlayerFromReefBacksideAuto(Pose2d targetPose, Pose2d startingPose) {
        Supplier<Pose2d> humanPlayerPose = () -> {
            double yError = Math.abs(getPose().getY() - targetPose.getY());

            double offset = 0.5;

            Rotation2d rotation = startingPose.getRotation();

            if (yError < 1.6) {
                offset = 0;
                rotation = targetPose.getRotation();
            }

            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                return new Pose2d(targetPose.getX() + offset, targetPose.getY(), rotation); 
            }
            else {
                return new Pose2d(targetPose.getX() - offset, targetPose.getY(), rotation);
            }
        };

        return Commands.deadline(Commands.waitUntil(() -> isAligned()), 
                Commands.run(() -> driveToPose(() -> new ChassisSpeeds(0, 0, 0), 
                    humanPlayerPose, PathLocation.HumanPlayer, 4), this))
            .finallyDo(() -> {
                this.targetPose = null;
                this.targetPoseLocation = PathLocation.None;
            });
    }

}
