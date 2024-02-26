package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class IntakePulley extends SubsystemBase {
    private CANSparkMax pulleyMotor;
    private RelativeEncoder pulleyMotorEncoder;
    private final SparkPIDController pulleyMotorPIDController;

  public IntakePulley() {
    pulleyMotor = new CANSparkMax(19,MotorType.kBrushless);
    pulleyMotor.setOpenLoopRampRate(.5);
    pulleyMotorEncoder = pulleyMotor.getEncoder();
    pulleyMotorPIDController = pulleyMotor.getPIDController();

   

    pulleyMotorPIDController.setP(1);
    pulleyMotorPIDController.setP(0);
    pulleyMotorPIDController.setP(0);
    pulleyMotorPIDController.setP(0);

  

  }
    
  public void moveIntakeIn(){
    pulleyMotor.set(Constants.IntakeConstants.pulleySpeed);
  }
  public void moveIntakeOut(){
    pulleyMotor.set(-Constants.IntakeConstants.pulleySpeed);
  }
  public void moveIntakeStop(){
    pulleyMotor.set(0);
  }





  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake pulley Position",pulleyMotorEncoder.getPosition());
}
}