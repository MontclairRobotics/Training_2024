package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private CANSparkMax topNEO;
    private CANSparkMax bottomNEO;
    
    public Intake () {
        topNEO = new CANSparkMax(Constants.IntakeConstants.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
        bottomNEO = new CANSparkMax(Constants.IntakeConstants.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
    }

    public void inTakeNote() {
        topNEO.set(Constants.IntakeConstants.INTAKE_SPEED);
        bottomNEO.set(-Constants.IntakeConstants.INTAKE_SPEED); //TODO: theres a negitive here but I'm not sure we need one
    }

    public void outTakeNote() {
        topNEO.set(-Constants.IntakeConstants.INTAKE_SPEED);
        bottomNEO.set(Constants.IntakeConstants.INTAKE_SPEED); //same here
    }

    public void stop(){
        topNEO.set(0);
        bottomNEO.set(0);
    }

    //Commands
    public Command inTakeNoteCommand(){
        return Commands.runOnce(()-> inTakeNote());
    }

    public Command outTakeNoteCommand(){
        return Commands.runOnce(()-> outTakeNote());
    }

    public Command stopCommand(){
        return Commands.runOnce(()-> stop());
    }
}