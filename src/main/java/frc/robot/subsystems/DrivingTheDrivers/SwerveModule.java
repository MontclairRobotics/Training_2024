package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private CANcoder canSparkCoder;
    private double driveVoltage;
    private double turnVoltage;
    private SwerveModuleState state;    

    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID) {
        
        
        canSparkCoder = new CANcoder(canCoderID,"rio"); //canbus must be named "rio"
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);
        
        RotationPID = new PIDController(0.3,0,0); //placeholder values for PID
        DrivePID  = new PIDController(0.3,0,0); //placeholder values for PID

        RotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setStateAndMove(SwerveModuleState moduleState) { //I think abe said that some varriables in here dont need to be fore the class but just for here better

        state = moduleState;
        
        driveVoltage = DrivePID.calculate(falconMotorDrive.getVelocity().getValueAsDouble()*Constants.DriveConstants.DRIVE_ENCODER_ROTATION_TO_METERS_OF_THE_WHEEL_RATIO, state.speedMetersPerSecond);
        turnVoltage = RotationPID.calculate(canSparkCoder.getPosition().getValue()*2*Math.PI, state.angle.getRadians());/*canSparkCoder.getPosition().getValue() is in rotations not radians so multiply by 2Pi*/ // This one needed .getRadians() because swervemodulestates stores a rotation 2d
        // TODO: Tune PID to get an output that is in voltages so we can put it in the move thing

        falconMotorDrive.setVoltage(driveVoltage);
        canTurnMotor.setVoltage(turnVoltage);
    }
}