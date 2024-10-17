package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private TalonFX falconMotorThing;
    private CANSparkMax turnMotor;
    private PIDController pid;
    private String canBusName;
    private CANcoder canCoder;
    private SwerveModuleState moduleState;
    private double tempXTarget = 10; //remove soon this represents the speeds were trying to get to and stuff (this is me writing code out probobly all wrong)
    private double tempYTarget = 10;
    private double tempRotationTarget = 90;
    private Rotation2d tempRotation2d = 30.0; //flip its a type mismach I don't know how they store the scary radian unit cirle robot rotation numbers! they have like a multible and then sin and cos right? Why must they do this to us! .·°՞(≧□≦)՞°·.
    
    public SwerveModule(int id) { //Hey wouldn't you need to ids or wait three? at least if we do it this way right? maybe I'm worng

        canCoder = new CANcoder(id,"useless");
        turnMotor = new CANSparkMax(id, MotorType.kBrushless);
        falconMotorThing = new TalonFX(id);
        pid = new PIDController(1,1,1);

        ChassisSpeeds.fromFieldRelativeSpeeds( //this "Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds object."
            tempXTarget, tempYTarget, tempRotationTarget, /*getRobotHeading()*/ tempRotation2d); //aww we need that method at some point (returns us a rotation 2d of where facing I think)
        // here we need an object of  SwerveDriveKinematics made with all of our Translation2d that cad has values for or somthing smh
        // SwerveModuleState[] swerveModuleStates =
        //     kinematics.toSwerveModuleStates(velocity, centerOfRotationMeters); // prety sure  all this stuff I'm doing goes in a different part of the file but I'll do real stuff tomawrrow this is just stuff abe told me about so now I'm writing it
                moduleState = new SwerveModuleState(); 
        
    }
}