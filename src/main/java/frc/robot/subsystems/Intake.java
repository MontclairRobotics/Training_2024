package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Intake {
    
    private TalonFX firstIntakeNeo;
    
    public Intake () {
        firstIntakeNeo = new TalonFX(98);
    }

    public void intakeNote() {
        firstIntakeNeo.set(Constants.IntakeConstans.intakeSpeed);
    }

    public void outtakeNote() {
        firstIntakeNeo.set(-Constants.IntakeConstans.intakeSpeed);
    }
}
