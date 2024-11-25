package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private CANcoder canCoder;
    private double driveVoltage;
    private double turnVoltage;
    private SwerveModuleState state;
    

    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID) {
        

        canCoder = new CANcoder(canCoderID,"rio"); //canbus must be named "rio"
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);

        RotationPID = new PIDController(0.5,0,0); //placeholder values for PID
        DrivePID  = new PIDController(0.5,0,0); //placeholder values for PID
    }

    public void move() { //I think abe said that some varriables in here dont need to be fore the class but just for here better
        
        
        driveVoltage = DrivePID.calculate(findCurrentSpeedHerePlease, state.speedMetersPerSecond); //TODO: currentDriveVoltage needs to be replaced with current speed cant mismatch units opps
        turnVoltage = RotationPID.calculate(canCoder.getPosition().getValue()/360, state.angle.getDegrees());/*canCoder.getPosition().getValue() is in rotations not deggres so i devided by 360 although we proboby want to change all our units later*/ // This one needed .getDegrees() because swervemodulestates stores a rotation 2d no degrees but I did some snooping in the class and found this.
        // TODO: Tune PID to get an output that is in voltages so we can put it in the move thing

        falconMotorDrive.setVoltage(driveVoltage);
        canTurnMotor.setVoltage(turnVoltage);
    }
    
    public void setState(SwerveModuleState moduleState) {
        state = moduleState;
    }
    
}