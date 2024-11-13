package frc.robot.subsystems.DrivingTheDrivers;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.PS5Controller;

public class SwerveModule {

    private TalonFX falconMotorDrive;
    private CANSparkMax canTurnMotor;
    private PIDController RotationPID;
    private PIDController DrivePID;
    private CANcoder canCoder;
    private double currentDriveVoltage;
    private double currentTurnVoltage;
    private SwerveModuleState state;

    
    public SwerveModule(int canCoderID, int canTurnMotorID, int falconMotorDriveID, int id) { //id: 0 front left, 1 front right, 2 back left, 3 back right.


        canCoder = new CANcoder(canCoderID,"rio"); //canbus must be named "rio"
        canTurnMotor = new CANSparkMax(canTurnMotorID, MotorType.kBrushless);
        falconMotorDrive = new TalonFX(falconMotorDriveID);


        RotationPID = new PIDController(1,1,1); //placeholder values for PID
        DrivePID  = new PIDController(1,1,1); //placeholder values for PID
    }

    public void move(double targetSpeed, double rotationDegrees) {
        
        
        currentDriveVoltage = DrivePID.calculate(currentDriveVoltage, state.speedMetersPerSecond); //omg its working. However, we have to replace that 0 with the ID we take in. I kinda forgot how to do that lol. I thought you could use the this keyword but then I got confused
        currentTurnVoltage = RotationPID.calculate(canCoder.getPosition().getValue(), state.angle.getDegrees()); // This one needed .getDegrees() because swervemodulestates stores a rotation 2d no degrees but I did some snooping in the class and found this.
        // TODO: Tune w/ voltage to get an output that is in voltages so we can put it in the move thing


        falconMotorDrive.set(currentDriveVoltage);
        canTurnMotor.set(currentTurnVoltage);
    }
    public void setState(SwerveModuleState moduleState) {
        state = moduleState;
    }

}