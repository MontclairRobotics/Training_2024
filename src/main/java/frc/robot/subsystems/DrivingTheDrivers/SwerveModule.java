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
        canCoder = new CANcoder(id1,"useless");
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
        // we need to tune :(
        m_pidController.setReference(1, CANSparkBase.ControlType.kVelocity);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative      
        falconMotorThing.getConfigurator().apply(slot0Configs); //configures stuff idk exactly
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        falconMotorThing.setControl(m_request.withVelocity(8).withFeedForward(0.5));
        // a lot of values will get changed, and I hope this is right
        
    }

    public Translation2d getPos() {
        return position;
    }
    
}