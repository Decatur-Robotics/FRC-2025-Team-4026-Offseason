package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private double voltage;
    private double velocity;
    private IntakeIOTalonFX motorLeft;

    private final String inputsName;
    private boolean isEStopped = false;
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
        private LinearFilter currentFilterLeft;
    private double filteredCurrentLeft;
    private LinearFilter currentFilterRight;
    private double filteredCurrentRight;


    static{
        switch(Constants.getRobotType()){
            case COMPETITION, OFFSEASON -> {

            }

            case SIMULATION -> {

            }
        }
    }

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputsName = this.getClass().getSimpleName() + "Inputs";
        currentFilterLeft = LinearFilter.movingAverage(10);
        currentFilterRight = LinearFilter.movingAverage(10);
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);

        if (isEStopped){
            io.stop();
        }
        // filteredCurrentLeft = currentFilterLeft.calculate(getCurrentLeft());
        // filteredCurrentRight = currentFilterRight.calculate(getCurrentRight());
        
    }



    public Command setVelocityCommand(double velocity){
        return Commands.runOnce(() -> io.setVelocity(velocity));
    }

    public Command setVoltageCommand(double volts){
        return Commands.runOnce(() -> io.setVoltage(volts));
    }

    // public double getCurrentLeft() {
    //     return inputs.data.leftMotorCurrent();
    // }
    
    // public double getCurrentRight() {
    //     return motorLeft.motorRight.getStatorCurrent().getValueAsDouble();
    // }

    public double getFilteredCurrentLeft() {
        return filteredCurrentLeft;
    }

    public double getFilteredCurrentRight() {
        return filteredCurrentRight;
    }
    
    public double getVelocity(){
        return inputs.intakeData.rightMotorVelocity();
    }


    
}
