// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  // The robot's subsystems and commands are defined here...

  private SwerveSubsystem drivebase;



  private final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

// Robot Relative
    // Commands.run(()->{
    //   var speeds = drivebase.joystickSpeeds(0, 0, 0);
    //   drivebase.setChasisSpeeds(speeds);
    // }, drivebase);

// Field Relative
    Commands.run(()->{
      var speeds = drivebase.joystickSpeeds(driver.getLeftX(), driver.getLeftY(), driver.getRightX());
      drivebase.setChasisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivebase.getAngle())
      );
    }, drivebase);


    driver.b().whileTrue(Commands.run(()->drivebase.setAngle(0),drivebase));

    driver.x().whileTrue(Commands.run(()->drivebase.lock(),drivebase));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(); // Does Nothing
  }
}
