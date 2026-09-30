package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  private SwerveDrive drive;
  private SwerveDriveConfig cfg; 

  @SuppressWarnings("static-access")
  public SwerveSubsystem()
  {
    cfg = new SwerveDriveConfig()
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        .withSubsystem(this)
        .withTelemetry("High",TelemetryVerbosity.HIGH)

        .withTranslationController(new PIDController(1.0, 0, 0)) // input: meters of position error
        .withRotationController(new PIDController(1.0, 0, 0));   // input: radians of heading error
    try
    {
      
      var parser = SwerveParser.parse(new File("swerve-config"));
      drive =  parser.createSwerveDrive(cfg);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y,
                                                    DoubleSupplier rot)
  {
    return new SwerveInputStream(drive, x, y, rot);
  }

  public Command drive(SwerveInputStream stream)
  {
    return drive.drive(() -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(),
                                                                   new Rotation2d(drive.getGyroAngle())));
  }

  /** Zero the gyro heading. Bind this to a button combo for field recovery. */
  public Command zeroGyro()
  {
    return runOnce(() -> drive.zeroGyro());
  }

  public Command driveToPose(Pose2d pose) {
  return drive.driveToPose(pose)
      .until(() -> drive.getDistanceFromPose(pose).in(Inches) < 1
                && Math.abs(drive.getAngleDifferenceFromPose(pose).in(Degrees)) < 2);
}

  public Command lock() {
    return run(()-> drive.lockPose());
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }

  @AutoLogOutput
  public ChassisSpeeds desiredSpeeds() {return drive.getDesiredChassisSpeeds();}

  @AutoLogOutput
  public ChassisSpeeds currentSpeeds() {return drive.getRobotRelativeSpeed();}

  @AutoLogOutput
  public Field2d field2d() {return drive.getField2d();}

  @AutoLogOutput
  public double gyroAngle() {return drive.getGyroAngle().in(Degrees);}
}