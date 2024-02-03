package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePIDController;

  public Intake() {
    intake = new CANSparkMax(16,MotorType.kBrushless);
    intakeEncoder = intake.getEncoder();
    intakePIDController = intake.getPIDController();

  }
  public void speedForward(){
    intake.set(Constants.IntakeConstants.intakeSpeed);
  }
  public void speedBack(){
    intake.set(-Constants.IntakeConstants.intakeSpeed);
  }
  public void speedStop(){
    intake.set(0);
  }
  @Override
  public void periodic() {

}
}
