package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Shooter extends SubsystemBase {
    private CANSparkMax shooter;
    private RelativeEncoder shooterEncoder;
    private final SparkPIDController shooterPIDController;

    private CANSparkMax feeder;
    private RelativeEncoder feederEncoder;
    private final SparkPIDController feederPIDController;

    private CANSparkMax hatch;
    private RelativeEncoder hatchEncoder;
    private final SparkPIDController hatchPIDController;

  public Shooter() {
    shooter = new CANSparkMax(17,MotorType.kBrushless);
    shooterEncoder = shooter.getEncoder();
    shooterPIDController = shooter.getPIDController();

    feeder = new CANSparkMax(18,MotorType.kBrushless);
    feederEncoder = feeder.getEncoder();
    feederPIDController = feeder.getPIDController();

    hatch = new CANSparkMax(21,MotorType.kBrushless);
    hatchEncoder = hatch.getEncoder();
    hatchPIDController = hatch.getPIDController();



  }
  public void shooterOn(){
    shooter.set(-Constants.ShooterConstants.shootSpeed);
  }
  public void shooterOff(){
    shooter.set(0);
  }
  public void shooterIntake(){
    shooter.set(Constants.ShooterConstants.intakeSpeed);
  }
  public void shooterAmp(){
    shooter.set(-Constants.ShooterConstants.ampSpeed);
  }

  public void feedForward(){
    feeder.set(-Constants.ShooterConstants.feedSpeed);
  }
  public void feedStop(){
    feeder.set(0);
  }
  public void hatchForward(){
    hatch.set(.1);
  }
  public void hatchBack(){
    hatch.set(-.1);
  }
  public void hatchStop(){
    hatch.set(0);
  }
  @Override
  public void periodic() {
}
}
