package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase{
    private double position;
    private double voltage;
    private double velocity;

    private boolean isEstopped;
    private ArmIOInputsAutoLogged inputs  = new ArmIOInputsAutoLogged();
}
