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
    private double tempXTarget = 10; //remove soon this represents the speeds were trying to get to and stuff (this is me writing code out probobly all wrong)
    private double tempYTarget = 10;
    private double tempRotationTarget = 90;
    private Rotation2d tempRotation2d = new Rotation2d(0,0);
    private Translation2d position;

    
    public SwerveModule(int id1, int id2, int id3, double x, double y) { //Hey wouldn't you need to ids or wait three? at least if we do it this way right? maybe I'm worng

        canCoder = new CANcoder(id1,"useless");
        turnMotor = new CANSparkMax(id2, MotorType.kBrushless);
        falconMotorThing = new TalonFX(id3);
        pid = new PIDController(1,1,1);
        position = new Translation2d(x,y);
        ChassisSpeeds.fromFieldRelativeSpeeds( //this "Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds object."
            tempXTarget, tempYTarget, tempRotationTarget, /*getRobotHeading()*/ tempRotation2d); //aww we need that method at some point (returns us a rotation 2d of where facing I think)
                moduleState = new SwerveModuleState(); 
        
    }

    public Translation2d getPos() {
        return position;
    }
    
}