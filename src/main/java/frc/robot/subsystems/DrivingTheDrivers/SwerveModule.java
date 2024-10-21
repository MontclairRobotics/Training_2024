package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private TalonFX falconMotorThing;
    private CANSparkMax turnMotor;
    private PIDController pid;
    private String canBusName;  
    private CANcoder canCoder;
    private SwerveModuleState moduleState;

    private Translation2d position;

    
    public SwerveModule(int id1, int id2, int id3, double x, double y) { //Hey wouldn't you need to ids or wait three? at least if we do it this way right? maybe I'm worng

        canCoder = new CANcoder(id1,"useless");
        turnMotor = new CANSparkMax(id2, MotorType.kBrushless);
        falconMotorThing = new TalonFX(id3);
        pid = new PIDController(1,1,1);
        position = new Translation2d(x,y);
        moduleState = new SwerveModuleState(); 
        
    }

    public Translation2d getPos() {
        return position;
    }
    
}