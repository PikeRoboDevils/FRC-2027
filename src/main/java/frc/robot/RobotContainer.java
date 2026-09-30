// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // Convert DriverXbox Raw data into ChasisSpeeds On Demand
  private final SwerveInputStream driveAngularVelocity =
      drivebase.getAngularVelocityStream(
                driverXbox::getLeftY,
                driverXbox::getLeftX,
                () -> driverXbox.getRawAxis(2))
            .withAllianceRelativeControl();

  /*
  * The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Default drive command
    drivebase.setDefaultCommand(drivebase.drive(driveAngularVelocity));
    
    // Reset gyro
    driverXbox.b().onTrue(drivebase.zeroGyro());
    
    // Lock Swerve, points wheels in an x shape
    driverXbox.x().whileTrue(drivebase.lock());

    // drive to pose Native to YAGSL   
    // driverXbox.leftBumper().onTrue(drivebase.driveToPose(new Pose2d(3, 3, Rotation2d.fromDegrees(30))));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Replace with Auto Command
    return Commands.none();
  }
}
