package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

public class ArmConstants {
    public static final double kA = 0.0;
    public static final double kV = 0.0;
    public static final double kP = 0.0;
    public static final double kS = 0.0;
    public static final double kD= 0.0;
    public static final double kG = 0.0;
    public static final double kI = 0.0;
    public static final double STOWED_POSITION = 0;


    public static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0.008));
}
