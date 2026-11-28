package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Swerve.m_backLeftLocation;
import static frc.robot.Constants.Swerve.m_backRightLocation;
import static frc.robot.Constants.Swerve.m_frontLeftLocation;
import static frc.robot.Constants.Swerve.m_frontRightLocation;
import static frc.robot.Constants.Swerve.maxAngularSpeed;
import static frc.robot.Constants.Swerve.maxSpeed;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.CanIDs;

public class SwerveSubsystem extends SubsystemBase {
    
  // FOR ALL CONFIGURATION USE NATIVE APPLICATIONS SparkMax -> Rev H.C. Krakens/Pigeon -> Pheonix Tuner
    
    private Pigeon2 pigeon2 = new Pigeon2(11);

    private Module fl,fr,bl,br;
    private Module[] ModuleIndex;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation );

    public Pose2d pose;

    private SwerveDriveOdometry m_odometry;
    /**
     * Creates New Swerve Drivebase 
     * Using the CanIDs set in Constants/Swerve/CanIDs  <p>
     * The physical Encoders & Motors expect to be configured before hand in their native applications 
     * Rev Hardware Client, Pheonix Tuner etc)
     */
    public SwerveSubsystem() {
        fl = new Module(CanIDs.flD,CanIDs.flT);
        fr = new Module(CanIDs.frD, CanIDs.frT);
        bl = new Module(CanIDs.blD,CanIDs.blT);
        br = new Module(CanIDs.brD,CanIDs.brT);

        ModuleIndex = new Module[]{fl,fr,bl,br};
        
        m_odometry = new SwerveDriveOdometry(
            kinematics,
            pigeon2.getRotation2d(),
            new SwerveModulePosition[] {fl.getPos(),fr.getPos(),bl.getPos(),br.getPos()}
        );
    }


    /**
     * A driving function for driving
     * @param x Translation X / Joystick Left X-axis [-1,1]
     * @param y Translation Y / Joystick Left Y-axis [-1,1]
     * @param theta Rotation Speed / Joystick Right X-axis [-1,1]
     * @return Desired ChasisSpeeds from joystick inputs
     */
    public ChassisSpeeds joystickSpeeds(double x, double y, double theta) {
        return new ChassisSpeeds(x*maxSpeed,y*maxSpeed,theta*maxAngularSpeed);
    }

    /**
     * Runs entire drivebase to given speeds / sets all modules to their desired speed <p>
     * It also Optimizize the given speeds before running them
     * @param speeds Desired ChassisSpeeds to set drivebase to
     */
    public void setChasisSpeeds(ChassisSpeeds speeds) {
        Logger.recordOutput("Swerve/Raw Chasis Speed", speeds);
        var moduleStates = kinematics.toSwerveModuleStates(speeds);
        Logger.recordOutput("Swerve/Modules/Raw Module Setpoints", moduleStates);

        for (int i = 0; i<ModuleIndex.length; i++) {
            // Optimize Turning
            moduleStates[i].optimize( new Rotation2d(ModuleIndex[i].getTurnPos()));

            // Cosine Compensation 
            moduleStates[i].speedMetersPerSecond *= 
                moduleStates[i]
                .angle
                .minus(new Rotation2d(ModuleIndex[i].getTurnPos())).getCos();

            // Set State
            ModuleIndex[i].setState( moduleStates[i]);
            Logger.recordOutput("Swerve/Optimized Module Setpoints", moduleStates[i]);
        }   
    }

    /**
     * 
     * @return the current robot state as ChasisSpeeds
     */
    public ChassisSpeeds getChasisSpeeds() {
        return kinematics.toChassisSpeeds(
                fl.getState(),
                fr.getState(),
                bl.getState(),
                br.getState());
    }

    /**
     * 
     * @return Heading of the drivebase in Rotation2d
     */
    public Rotation2d getAngle() {
       return new Rotation2d(pigeon2.getYaw().getValue());  
    }

    /**
     * Reset the Heading of the drivebase 
     * this creates an offset in Odometry
     * @param theta The angle the robot should be set to
     */
    public void setAngle(double theta) {
    //    pigeon2.setYaw(theta); // Will mess up Odometry
    resetPose(new Pose2d(pose.getTranslation(), new Rotation2d(theta)));
    }

    /**
     * Locks the drivebase <p>
     * Hard-Coded to set all wheels to face inwards and slowly move against each other 
     * making it harder to push the robot around
     */
    public void lock() {
        setChasisSpeeds(
            kinematics.toChassisSpeeds(
                new SwerveModuleState(.1,Rotation2d.fromDegrees(225)),
                new SwerveModuleState(.1,Rotation2d.fromDegrees(135)),
                new SwerveModuleState(.1,Rotation2d.fromDegrees(315)),
                new SwerveModuleState(.1,Rotation2d.fromDegrees(45)))
        );
    }

    /**
     * Resets the Pose 
     * @param nPose
     */
    public void resetPose(Pose2d nPose) { 
        m_odometry.resetPosition(getAngle(),new SwerveModulePosition[] {fl.getPos(),fr.getPos(),bl.getPos(),br.getPos()},
         nPose);
    }

    @Override
    public void periodic() {
    // Periodically send a set of module states
    pose = m_odometry.update(getAngle(), new SwerveModulePosition[] {fl.getPos(),fr.getPos(),bl.getPos(),br.getPos()});


    Logger.recordOutput("Swerve/Modules/Current Module States",
        fl.getState(),
        fr.getState(),
        bl.getState(),
        br.getState());

    Logger.recordOutput("Swerve/pose", pose);
    Logger.recordOutput("Swerve/Yaw", getAngle());
    Logger.recordOutput("Swerve/Current Chasis Speeds",getChasisSpeeds());

    
    }



}
