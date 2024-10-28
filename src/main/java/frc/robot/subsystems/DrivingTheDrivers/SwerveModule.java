package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

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

    
    public SwerveModule(int id1, int id2, int id3, double x, double y) { 
        // We also need to put the data (swervestates) into here so we know the setpoint/request with velocity
        canCoder = new CANcoder(id1,"rio");
        turnMotor = new CANSparkMax(id2, MotorType.kBrushless);
        falconMotorThing = new TalonFX(id3);
        pid = new PIDController(1,1,1);
        position = new Translation2d(x,y);
        moduleState = new SwerveModuleState(); 
        // we need revpid controls :)!!!
        SparkPIDController m_pidController = turnMotor.getPIDController();
        m_pidController.setP(0.5);
        m_pidController.setI(0.5);
        m_pidController.setD(0.5);
        m_pidController.setIZone(0.5);
        // we need to tune w/ voltage to get an output that is in voltages so we can put it in the move thing
        m_pidController.setReference(1, CANSparkBase.ControlType.kVelocity);
        // a lot of values will get changed, and I hope this is right, and we also if we wanted we can put some of these in methods to clean up the code but we don't need to
        
    }

    public void move(double falconVoltage, double turnVoltage) {  // this should acutally work the one thing we have to do now is get the modual state into the the PID. LET's Do this!
        falconMotorThing.setVoltage(falconVoltage);
        turnMotor.set(turnVoltage);
    }
   
}