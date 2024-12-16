package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private CANSparkMax topIntakeNeo;
    private CANSparkMax bottomIntakeNeo;
    
    public Intake () {
        topIntakeNeo = new CANSparkMax(Constants.IntakeConstants.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
        bottomIntakeNeo = new CANSparkMax(Constants.IntakeConstants.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
    }

    public void inTakeNote() {
        topIntakeNeo.set(Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeNeo.set(-Constants.IntakeConstants.INTAKE_SPEED); //TODO: theres a negitive here but I'm not sure we need one
    }

    public void outTakeNote() {
        topIntakeNeo.set(-Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeNeo.set(Constants.IntakeConstants.INTAKE_SPEED); //same here
    }

    public void stop(){
        topIntakeNeo.set(0);
        bottomIntakeNeo.set(0);
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