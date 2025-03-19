package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private SparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private CANcoder canCoder;
    public double driveVoltage;
    public double turnVoltage;
    public double canCoderOffSet;
    
    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID, double canCoderOffSet) {
        
        
        canCoder = new CANcoder(canCoderID,"rio"); //"rio" is the default canbus of the roborio
        canTurnMotor = new SparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);
        
        RotationPID = new PIDController(2,0,0); //placeholder values for PID
        DrivePID  = new PIDController(5.7,0,0); //placeholder values for PID
        RotationPID.enableContinuousInput(-Math.PI, Math.PI);
        // This is needed because in a circle -180 is the same as +180
        this.canCoderOffSet = canCoderOffSet;
    }
    
    public void setStateAndMove(SwerveModuleState moduleState) {
        
        double currentVelocity = falconMotorDrive.getVelocity().getValueAsDouble()*Constants.DriveConstants.DRIVE_ENCODER_ROTATION_TO_METERS_OF_THE_WHEEL_RATIO;
        //would give us value thats how many turns of the encoder but we want it in meters/second so we multiply by the constant
        double currentRotation = canCoder.getPosition().getValueAsDouble()*2*Math.PI - canCoderOffSet;
        // canSparkCoder.getPosition().getValue() is in rotations not radians so multiply by 2Pi (aka 360 degrees but we use radians). Then because the cancoder is an absolute encoder there is an offset
        
        moduleState.optimize(Rotation2d.fromRadians(currentRotation)); 
        //optimizes so that will turn in the closest direction to get to target

        driveVoltage = DrivePID.calculate(currentVelocity, moduleState.speedMetersPerSecond); //set Drive voltage using PID(current velocity, target velocity)
        
        turnVoltage = RotationPID.calculate(currentRotation, moduleState.angle.getRadians()); //set turn voltage using PID(current rotation, target rotation)
        // .angle needed .getRadians() because swerveModuleStates stores a rotation 2d

        falconMotorDrive.setVoltage(driveVoltage); //tells the motors to move
        canTurnMotor.setVoltage(turnVoltage);
    }
}