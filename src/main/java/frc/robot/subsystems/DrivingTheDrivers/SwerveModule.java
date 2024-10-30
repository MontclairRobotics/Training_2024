package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Rotation2d; ////were not using this here are we?
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds; //were not using this here are we?
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController pid;
    private CANcoder canCoder; //wholy cow my java must be gone because why is it say that these go unused???? I'm probobly blind
    private SwerveModuleState moduleState;
    private Translation2d position;

    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID, double xTranslation2d, double yTranslation2d) { 
        // We also need to put the data (swervestates) into here so we know the setpoint/request with velocity
        canCoder = new CANcoder(canCoderID,"rio"); //changed name to rio here (must be called rio can't just make up a name)
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);
        pid = new PIDController(1,1,1);
        position = new Translation2d(xTranslation2d,yTranslation2d);
        moduleState = new SwerveModuleState(); 
        // TODO: we need revpid controls :)!!!
        // what does the above comment mean? I have no clue
        SparkPIDController m_pidController = canTurnMotor.getPIDController(); //hmm just noticed that this says to do cansparkbase.getpidcontroller to make a pid controller but I don't know.
        m_pidController.setP(0.5); //  also not sure what the M_PID is? maybe can explain to me? (not that I looked)
        m_pidController.setI(0.5); //I gota review documentation about pid to Find out how to put the swerve modual state in and get the voltage out and also figure out how to use sisID or whaterver its called.
        m_pidController.setD(0.5);
        m_pidController.setIZone(0.5);
        // TODO: we need to tune w/ voltage to get an output that is in voltages so we can put it in the move thing
        m_pidController.setReference(1, CANSparkBase.ControlType.kVelocity);
        //TODO: gota fix this mess later         
    }

    public void move(double falconVoltage, double turnVoltage) {  // this should acutally work the one thing we have to do now is get the modual state into the the PID. LET's Do this!
        falconMotorDrive.setVoltage(falconVoltage);
        canTurnMotor.set(turnVoltage);
    }
   
}