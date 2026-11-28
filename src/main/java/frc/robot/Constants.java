// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;

    /**
   * if Replaying a Log change from simMode to REPLAY
   */
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {

    public static class CanIDs {

      public static int flD = 0;
      public static int flT = 0;
      
      public static int frD = 0;
      public static int frT = 0;

      public static int blD = 0;
      public static int blT = 0;
      
      public static int brD = 0;
      public static int brT = 0;     

    }

    public static final double driveP = 1;

    public static final double turnP = 1;
    public static final double turnI = 0;
    public static final double turnD = 0.1;

    public static final double maxSpeed = 4.60248; // Meters per Sec
    public static final double maxAngularSpeed = 4*Math.PI; // Radians per Sec

    // Locations for the swerve drive modules relative to the robot center.
      private static final double CenToEdge = 0.3429; // From Center(gyro) to Edge(wheel X OR Y) of robot METERS
  public static final Translation2d m_frontLeftLocation = new Translation2d(+CenToEdge, +CenToEdge); // +,+ 
  public static final Translation2d m_frontRightLocation = new Translation2d(+CenToEdge, -CenToEdge);
  public static final Translation2d m_backLeftLocation = new Translation2d(-CenToEdge, +CenToEdge);
  public static final Translation2d m_backRightLocation = new Translation2d(-CenToEdge, -CenToEdge);
  }

}
