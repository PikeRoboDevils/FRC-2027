package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Swerve.*;

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

public class SwerveSubsystem extends SubsystemBase {
    
  // FOR ALL CONFIGURATION USE NATIVE APPLICATIONS SparkMax -> Rev H.C. Krakens/Pigeon -> Pheonix Tuner
    
    private Pigeon2 pigeon2 = new Pigeon2(11);

    private Module fl,fr,bl,br;
    private Module[] ModuleIndex= {fl,fr,bl,br};
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation );

    public Pose2d pose;
    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        kinematics,
        pigeon2.getRotation2d(),
        new SwerveModulePosition[] {
            fl.getPos(),fr.getPos(),bl.getPos(),br.getPos()});

    public SwerveSubsystem() {
        fl = new Module(0, 0);
        fr = new Module(0, 0);
        bl = new Module(0, 0);
        br = new Module(0, 0);
    }


    public ChassisSpeeds joystickSpeeds(double x, double y, double theta) {
        return new ChassisSpeeds(x*maxSpeed,y*maxSpeed,theta*maxAngularSpeed);
    }

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

    public ChassisSpeeds getChasisSpeeds() {
        return kinematics.toChassisSpeeds(
                fl.getState(),
                fr.getState(),
                bl.getState(),
                br.getState());
    }

    public Rotation2d getAngle() {
       return new Rotation2d(pigeon2.getYaw().getValue());  
    }

    public void setAngle(double theta) {
    //    pigeon2.setYaw(theta); // Will mess up Odometry
    resetPose(new Pose2d(pose.getTranslation(), new Rotation2d(theta)));
    }

    public void lock() {
        setChasisSpeeds(
            kinematics.toChassisSpeeds(
                new SwerveModuleState(.1,Rotation2d.fromDegrees(225)),
                new SwerveModuleState(.1,Rotation2d.fromDegrees(135)),
                new SwerveModuleState(.1,Rotation2d.fromDegrees(315)),
                new SwerveModuleState(.1,Rotation2d.fromDegrees(45)))
        );
    }

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
