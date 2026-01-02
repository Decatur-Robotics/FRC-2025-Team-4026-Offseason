package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Wrist extends SubsystemBase{
    private final String inputsName;
    private final WristIO io;
    private double volts = 0.0;
    protected final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private WristIOTalonFX wristMotor;
    private Debouncer slamDebouncer;
    private double filteredVelocity;
    private Boolean isSlammed;

    
    private boolean brakeModeEnabled = true;
    public Wrist(String inputsName, WristIO io) {

        this.inputsName = inputsName;
        this.io = io;
        slamDebouncer = new Debouncer(WristConstants.SLAM_DEBOUNCE_TIME);

        isSlammed = false;

    }
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);


        io.setVolts(volts);
        isSlammed = slamDebouncer.calculate(Math.abs(filteredVelocity) < WristConstants.MAX_SLAMMED_VELOCITY);

        //LoggedTracer.record(name);
        // this doesn't work mayhbe important idk 
        Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(enabled);
        

    }

    public double getVolts() {
        return inputs.data.appliedVolts();
    }
    public double getPosition() {
        return inputs.data.positionRad();
    }
    
    
  

    public double getCurrent() {
        return inputs.data.supplyCurrentAmps();
    }
    
    public Command setVoltsCommand(double volts) {
        return Commands .runOnce(() -> io.setVolts(volts));
    }

    public Command setCurrentCommand(double current) {
        return Commands.runOnce(() -> io.setCurrent(current));
    }

    
    public boolean isSlammed() {
        return isSlammed;
    }

    
    
}