package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Rotation2d; ////were not using this here are we?
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds; //were not using this here are we?
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private CANcoder canCoder; //wholy cow my java must be gone because why is it say that these go unused???? I'm probobly blind
    private SwerveModuleState moduleState;
    private Translation2d translation2d;
    private double currentDriveVoltage;
    private double currentTurnVoltage;

    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID) { 

        // We also need to put the data (swervestates) into here so we know the setpoint/request with velocity
         //wait what do we?

        canCoder = new CANcoder(canCoderID,"rio"); //changed name to rio here (must be called rio can't just make up a name)
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);

        RotationPID = new PIDController(1,1,1); //placeholder values for PID
        DrivePID  = new PIDController(1,1,1); //placeholder values for PID
// TODO: we need to tune w/ voltage to get an output that is in voltages so we can put it in the move thing

        moduleState = new SwerveModuleState(); 
    }

    public void move(double targetSpeed, double rotationDegrees) {
        
        currentDriveVoltage = DrivePID.calculate(currentDriveVoltage, targetSpeed);
        currentTurnVoltage = RotationPID.calculate(canCoder.getPosition().getValue(), rotationDegrees);
        falconMotorDrive.set(currentDriveVoltage);
        canTurnMotor.set(currentTurnVoltage);
    }
   
}