package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public class IntakeConstants {

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.349;
    public static final double kV = 0.1;
    public static final double kA = 0.0;
    
    public static final double CORAL_REST_VELOCITY = 0;
    public static final double ALGAE_REST_VELOCITY = 120;
    public static final double CORAL_INTAKE_VELOCITY = 120;
    public static final double ALGAE_INTAKE_VELOCITY = 120;
    public static final double ALGAE_REMOVE_VELOCITY = -80;
    public static final double L1_EJECT_VELOCITY = -10;
    public static final double BRANCH_EJECT_VELOCITY = -15;
    public static final double PROCESSOR_EJECT_VELOCITY = -120;
    public static final double NET_EJECT_VELOCITY = -120;
    public static final double DEALGIFY_VELOCITY = -60;

    public static final double CORAL_STALL_DEBOUNCE_TIME = 0.1;
    public static final int CORAL_STALL_CURRENT = 50;

    public record IntakeHardwareConstants(
        Distance INTAKE_WIDTH,
        Mass INTAKE_MASS,
        Distance INTAKE_MAX_EXTENSION,
        DCMotor INTAKE_GEARBOX,
        double INTAKE_GEARING_REDUCTION
    ) {
    }

    public static final IntakeHardwareConstants HARDWARE_CONSTANTS = new IntakeHardwareConstants(
        Distance.ofBaseUnits(0.1, Meters),
        Mass.ofBaseUnits(3, Kilograms),
        Distance.ofBaseUnits(0.2, Meters),
        DCMotor.getKrakenX60Foc(2),
        10.0
    );
}
