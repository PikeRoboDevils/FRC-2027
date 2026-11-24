package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Swerve.*;

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

    /* Initialize Module with Drive motor and Turn Motor CanIDS */
    public Module(int DriveID, int TurnID){

        Dmotor = new SparkMax(DriveID, MotorType.kBrushless);
        Tmotor = new SparkMax(TurnID, MotorType.kBrushless);

        // MOTOR CONFIG

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

        Dconfig.closedLoop.outputRange(0,maxSpeed);
        Tconfig.closedLoop.outputRange(-2*Math.PI,2*Math.PI);

        Tconfig.closedLoop.dFilter(0.1);

        Dconfig.closedLoop.positionWrappingEnabled(false);
        Tconfig.closedLoop.positionWrappingEnabled(true);

        Dconfig.closedLoop.pid(driveP, 0, 0);
        Tconfig.closedLoop.pid(turnP, turnI, turnD);


        // RESET & PERSIST
        Dmotor.configure(Dconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Tmotor.configure(Tconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void setDriveVel(double velocity) {
        Dmotor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
    }

    private void setTurnPos(double position) {
        Dmotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    public double getTurnPos() {
        return Tmotor.getAbsoluteEncoder().getPosition();
    }

    public SwerveModulePosition getPos() {
        return new SwerveModulePosition(
            Dmotor.getEncoder().getPosition(),
             new Rotation2d(getTurnPos()));
    }

    public void setState(SwerveModuleState state) {
        setDriveVel(state.speedMetersPerSecond);
        setTurnPos(state.angle.getRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Dmotor.getEncoder().getVelocity(),
            new Rotation2d(Tmotor.getAbsoluteEncoder().getPosition()));
    }
}
