package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.19;
    public static final double kV = 0.13;
    public static final double kA = 0.007;
    public static final double kG = 0.39;
    public static final double STOWED_POSITION = 0;

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = 0;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = 4.25;
    public static final double ALGAE_GROUND_INTAKING_POSITION = 0;
    public static final double ALGAE_LOW_REEF_INTAKING_POSITION = 19.47;
    public static final double ALGAE_HIGH_REEF_INTAKING_POSITION = 34.63;
    public static final double ALGAE_LOW_REEF_REMOVING_POSITION = 9.47;
    public static final double ALGAE_HIGH_REEF_REMOVING_POSITION = 26.63;

    // Scoring positions
    public static final double L1_POSITION = 0;
    public static final double STAGE_L2_POSITION = 12.52;
    public static final double SCORE_L2_POSITION = 0.02;
    public static final double STAGE_L3_POSITION = 30.18;
    public static final double SCORE_L3_POSITION = 17.68;
    public static final double STAGE_L4_POSITION = 54.5;
    public static final double SCORE_L4_POSITION = 43.5;
    public static final double PROCESSOR_POSITION = 0;
    public static final double NET_POSITION = 54;

     public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);

    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
    public record ElevatorHardwareConstants(
        Distance CHAIN_LENGTH,
            int ELEVATOR_DRUM_WHEEL_TEETH,
            int ELEVATOR_STAGES,
            double ELEVATOR_GEARING_REDUCTION,
            DCMotor ELEVATOR_GEARBOX,
            Mass ELEVATOR_CARRIAGE_WEIGHT,
            Distance ELEVATOR_MAX_HEIGHT
    ) {
    }

    public static final ElevatorHardwareConstants HARDWARE_CONSTANTS = new ElevatorHardwareConstants(
        Distance.ofBaseUnits(1.32, Meters),
            22,
            2,
            15.0,
            DCMotor.getFalcon500Foc(2),
            Mass.ofBaseUnits(9, Kilograms),
            Distance.ofBaseUnits(2.3, Meters)
    );

}
