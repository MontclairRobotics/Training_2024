package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake {
    
    private TalonFX firstIntakeNeo;
    
    public Intake () {
        firstIntakeNeo = new TalonFX(98);
    }

    public void intakeNote() {
        firstIntakeNeo.set(0.8);
    }

    public void outtakeNote() {
        firstIntakeNeo.set(-0.8);
    }
}
