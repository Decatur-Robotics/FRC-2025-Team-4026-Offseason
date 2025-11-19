package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.wrist.Wrist;

public class Superstructure {
    private Elevator elevator;
    private Arm arm;
    private Wrist wrist;
    private Intake intake;
    public Superstructure(Elevator elevator, Arm arm, Intake intake, Wrist wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
    }
}
