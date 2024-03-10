package frc.robot.Subsystems;

import java.util.Optional;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS m_gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private SwerveDrivePoseEstimator swervePoseEstimator;

  private AprilTagFieldLayout fieldLayout;
  public Field2d field;


  

  public Swerve() {
    m_gyro = new AHRS(SPI.Port.kMXP);
    //.configFactoryDefault();
    zeroHeading();
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRotation2d(), getModulePositions());

    field = new Field2d();
    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(.1, .1, .1);
  
     
    swervePoseEstimator =
            new SwerveDrivePoseEstimator(
                    Constants.Swerve.swerveKinematics,
                    getRotation2d(),
                    getModulePositions(),
                    new Pose2d(),
                    stateStdDevs,
                    visionStdDevs);



    SmartDashboard.putData("Field", field);
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // Translation PID constants
            new PIDConstants(5, Constants.Swerve.angleKI, Constants.Swerve.angleKD), // Rotation PID constants
            4, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
  public void drive(Translation2d translation, double rotation, /*boolean fieldRelative,*/ boolean isOpenLoop) {
      SwerveModuleState[] swerveModuleStates = 
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2d()));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    //return swerveOdometry.getPoseMeters();
    return swervePoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    //swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    swervePoseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    return chassisSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }
  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(targetStates[mod.moduleNumber], true);
    }
  }
  
  public void resetOdometry(Pose2d pose) {
    //swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    swervePoseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void zeroHeadingAdjust() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }
  public void setHeading(){
    m_gyro.setAngleAdjustment(180);
  }

  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
  }
  public double getGyroRoll(){
    return m_gyro.getRoll();
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void resetModulesToAbsolute() {
      for(SwerveModule mod : mSwerveMods) {
        mod.resetToAbsolute();
        System.out.println("Modules Reset to Absolute");
      }
  }
 
  
  public void runVolts(Measure<Voltage> voltage)  {    
    for (SwerveModule mod : mSwerveMods) {
      mod.runVolts(voltage);
    }
  }
 
  @Override
  public void periodic() {
    swervePoseEstimator.update(getRotation2d(), getModulePositions());
    
     // Correct pose estimate with vision measurements
    SmartDashboard.putNumber("poseY", field.getRobotPose().getY());
    SmartDashboard.putNumber("NAVX Heading", this.getHeading());


  
    field.setRobotPose(getPose());

    //if(result.isPresent()){
    //  SmartDashboard.putNumber("Vision Estimate X", result.get().estimatedPose.toPose2d().getX());
    //  SmartDashboard.putNumber("Vision Estimate Y", result.get().estimatedPose.toPose2d().getY());
    //}

   
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    

  }
}