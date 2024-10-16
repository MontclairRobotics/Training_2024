package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private TalonFX falconMotorThing;
    private PIDController pid;
    private String canBusName;
    private CANcoder canCoder;
    private SwerveModuleState moduleState;
    
    public SwerveModule(int id) {
        canCoder = new CANcoder(id,"useless");
        turn = new CANSparkMax(id, MotorType.kBrushless);
        falconMotorThing = new TalonFX(id);
        pid = new PIDController(1,1,1);
    }
  
    
    

    public void drive(double speed){
        // this.speed = speed;
        // falconMotorThing.set(pid.calculate(canCoder.getAbsolutePosition(), speed));
    }

    public void turn(double angle) {
        // this.angle = angle;
        // turn.set(pid.calculate(canCoder.getAbsolutePosition(), angle));
    }
    //  PID controller for these motors
    // cancoders
    
}
    