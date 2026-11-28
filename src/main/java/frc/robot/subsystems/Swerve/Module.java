package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Swerve.driveP;
import static frc.robot.Constants.Swerve.maxSpeed;
import static frc.robot.Constants.Swerve.turnD;
import static frc.robot.Constants.Swerve.turnI;
import static frc.robot.Constants.Swerve.turnP;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private SparkMax Dmotor, Tmotor;
    private SparkMaxConfig Dconfig, Tconfig;

    /**
     *  Creates a New Module
     * with CanIDs for SparkMaxes 
     * 
     * Inversions and Encoder Offsets should be set BEFOREHAND
     * INSIDE the REV HARDWARE CLIENT
     *  
     * @param DriveID Drive Motor CanID
     * @param TurnID Turn Motor CanID 
     * 
     * @return A Module Object
     * 
     * */
    public Module(int DriveID, int TurnID){

        Dmotor = new SparkMax(DriveID, MotorType.kBrushless);
        Tmotor = new SparkMax(TurnID, MotorType.kBrushless);

        // MOTOR CONFIG

        Dconfig = new SparkMaxConfig();
        Tconfig = new SparkMaxConfig();

        Dconfig.smartCurrentLimit(20,30);
        Tconfig.smartCurrentLimit(20,30);

        Dconfig.secondaryCurrentLimit(40);
        Tconfig.secondaryCurrentLimit(40);

        Dconfig.voltageCompensation(12);
        Tconfig.voltageCompensation(12);

        Dconfig.idleMode(IdleMode.kCoast);
        Tconfig.idleMode(IdleMode.kBrake);

            Dconfig.encoder.positionConversionFactor(1/6.75); //MK4i L2 Gearing
            Dconfig.encoder.velocityConversionFactor(1/6.75); //MK4i L2 Gearing
            Tconfig.absoluteEncoder.positionConversionFactor( 1/(150/7) ); // MK4i steering gear ratio

        Dconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        Tconfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        Dconfig.closedLoop.outputRange(-maxSpeed,maxSpeed);
        Tconfig.closedLoop.outputRange(-2*Math.PI,2*Math.PI);

        Tconfig.closedLoop.dFilter(0.1);

        Dconfig.closedLoop.positionWrappingEnabled(false);
        Tconfig.closedLoop.positionWrappingEnabled(true);

        Dconfig.closedLoop.pid(driveP, 0, 0);
        Tconfig.closedLoop.pid(turnP, turnI, turnD);


        // NO RESET & PERSIST
        Dmotor.configure(Dconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        Tmotor.configure(Tconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void setDriveVel(double velocity) {
        // Dmotor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
        Dmotor.set(velocity/maxSpeed);
    }

    private void setTurnPos(double position) {
        Tmotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    /** 
     * @return Turn Motor Position of this Module in RADIANS
     */
    public double getTurnPos() {
        return Tmotor.getAbsoluteEncoder().getPosition();
    }

    /**
     * FOR ODOMETRY  <p>
     * the "position" of this module for ODOMETRY 
     * this gives the Total drive distance and Current angle of the module
     * @return  SwerveModulePosition Object for this Module
     */
    public SwerveModulePosition getPos() {
        return new SwerveModulePosition(
            Dmotor.getEncoder().getPosition(),
             new Rotation2d(getTurnPos()));
    }

    /**
     * Drives motors to match given ModuleState 
     * @param state a WPILIB object giving both Drive Velocity and Turn Position
     */
    public void setState(SwerveModuleState state) {
        setDriveVel(state.speedMetersPerSecond);
        setTurnPos(state.angle.getRadians());
    }

    /**
     * 
     * @return  The current state of this Module <p> Drive Velocity + Angle (Radians) 
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Dmotor.getEncoder().getVelocity(),
            new Rotation2d(Tmotor.getAbsoluteEncoder().getPosition()));
    }
}
