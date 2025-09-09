package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Wrist extends WristIOTalonFX {
    private LinearFilter velocityFilter;
    private double filteredVelocity;

    private Debouncer slamDebouncer;

    private Boolean isSlammed;

    public Wrist() {
        velocityFilter = LinearFilter.movingAverage(10);

        slamDebouncer = new Debouncer(WristConstants.SLAM_DEBOUNCE_TIME);

        isSlammed = false;


    }

    public void setCurrent(double current) {
        this.current = current;
        motor.setControl(controlRequest.withOutput(current));
    }

    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public boolean isSlammed() {
        return isSlammed;
    }

    public Command setCurrentCommand(double current) {
        return Commands.runEnd(() -> setCurrent(current), 
            () -> setCurrent(0),
            this);
    }
}
