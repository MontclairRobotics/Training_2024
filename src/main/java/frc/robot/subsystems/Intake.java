package frc.robot.subsystems;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private SparkMax topNEO;
    private SparkMax bottomNEO;
    
    public Intake () {
        topNEO = new SparkMax(Constants.IntakeConstants.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
        bottomNEO = new SparkMax(Constants.IntakeConstants.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
    }

    public void inTakeNote() {
        topNEO.set(Constants.IntakeConstants.INTAKE_SPEED);
        bottomNEO.set(-Constants.IntakeConstants.INTAKE_SPEED);
    }

    public void outTakeNote() {
        topNEO.set(-Constants.IntakeConstants.INTAKE_SPEED);
        bottomNEO.set(Constants.IntakeConstants.INTAKE_SPEED);
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