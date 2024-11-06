package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d; //were not using this here are we?
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private SwerveModuleState[] swerveModuleState; //TODO: figure out why this says not being used all wierd.
    private CANcoder canCoder;
    private Translation2d translation2d;
    private double currentDriveVoltage;
    private double currentTurnVoltage;
    private double tempXTarget = 10; //remove soon this represents the speeds were trying to get to and stuff (this is me writing code out probobly all wrong)
    private double tempYTarget = 10;
    private double tempRotationTarget = 90;
    private Rotation2d tempRotation2d = new Rotation2d(0,0);

    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID) { 

        // We also need to put the data (swervestates) into here so we know the setpoint/request with velocity
         //wait what do we?

        canCoder = new CANcoder(canCoderID,"rio"); //changed name to rio here (must be called rio can't just make up a name)
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);

        

        RotationPID = new PIDController(1,1,1); //placeholder values for PID
        DrivePID  = new PIDController(1,1,1); //placeholder values for PID
// TODO: we need to tune w/ voltage to get an output that is in voltages so we can put it in the move thing

    }

    public void move(double targetSpeed, double rotationDegrees) {

        swerveModuleState = RobotContainer.drive.roboSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            tempXTarget, tempYTarget, tempRotationTarget, tempRotation2d)); //TODO: replace temps
        
        currentDriveVoltage = DrivePID.calculate(currentDriveVoltage, swerveModuleState);
        currentTurnVoltage = RotationPID.calculate(canCoder.getPosition().getValue(), rotationDegrees);
        
        falconMotorDrive.set(currentDriveVoltage);
        canTurnMotor.set(currentTurnVoltage);
    }
   
}