package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private CANcoder canCoder;
    private double driveVoltage;
    private double turnVoltage;
    private double canCoderOffSet;
    
    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID, double canCoderOffSet) {
        
        
        canCoder = new CANcoder(canCoderID,"rio"); //canbus must be named "rio"
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);
        
        RotationPID = new PIDController(0.4,0,0); //placeholder values for PID
        DrivePID  = new PIDController(0.4,0,0); //placeholder values for PID

        RotationPID.enableContinuousInput(-Math.PI, Math.PI);
        // This is needed because its a circle if it gets to -180 thats the same as +180
        this.canCoderOffSet = canCoderOffSet;
    }

    public void setStateAndMove(SwerveModuleState moduleState) {
        
        double currentVolocity = falconMotorDrive.getVelocity().getValueAsDouble()*Constants.DriveConstants.DRIVE_ENCODER_ROTATION_TO_METERS_OF_THE_WHEEL_RATIO;
        //would give us value thats how many turns of the encoder but we want it in meters/second so we multiply by the constant
        double currentRotation = canCoder.getPosition().getValue()*2*Math.PI - canCoderOffSet;
        // canSparkCoder.getPosition().getValue() is in rotations not radians so multiply by 2Pi (aka 360 degrees but we use radians). Then because the cancoder is an absalute encoder there is an offset

        SwerveModuleState optimizedState = SwerveModuleState.optimize(moduleState, Rotation2d.fromRadians((currentRotation))); 
        //optimizes so that will turn in the closest direction to get to target

        // TODO: Tune PID to get an output that is in voltages so we can put it in the move thing
        driveVoltage = DrivePID.calculate(currentVolocity, optimizedState.speedMetersPerSecond); //set Drive volatage using PID(current volocity, target volocity)
        
        turnVoltage = RotationPID.calculate((currentRotation), optimizedState.angle.getRadians()); //set turn volatage using PID(current rotarion, target rotation)
        // .angle needed .getRadians() because swervemodulestates stores a rotation 2d

        falconMotorDrive.setVoltage(driveVoltage); //tells the motors to move
        canTurnMotor.setVoltage(turnVoltage);
    }
}