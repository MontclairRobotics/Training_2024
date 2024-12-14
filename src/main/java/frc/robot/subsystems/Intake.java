package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake {
    
    private TalonFX firstIntakeNeo;
    
    public Intake () {
        firstIntakeNeo = new TalonFX(98);
        firstIntakeNeo.set(0.8);

    }
}
