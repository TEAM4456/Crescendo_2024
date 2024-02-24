package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ShooterPivot extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotMotorEncoder;
    private final SparkPIDController pivotMotorPIDController;

  public ShooterPivot() {
    pivotMotor = new CANSparkMax(20,MotorType.kBrushless);
    pivotMotor.setOpenLoopRampRate(.5);
    pivotMotorEncoder = pivotMotor.getEncoder();
    pivotMotorPIDController = pivotMotor.getPIDController();

   

    pivotMotorPIDController.setP(1);
    pivotMotorPIDController.setP(0);
    pivotMotorPIDController.setP(0);
    pivotMotorPIDController.setP(0);

  

  }
    
  public void ShooterPivotUp(){
    pivotMotor.set(Constants.ShooterPivotPositions.shooterPivotSpeed);
    pivotMotor.set(Constants.ShooterPivotPositions.shooterPivotSpeed);
  }
  public void ShooterPivotDown(){
    pivotMotor.set(-Constants.ShooterPivotPositions.shooterPivotSpeed);
    pivotMotor.set(-Constants.ShooterPivotPositions.shooterPivotSpeed);
  }
  public void ShooterPivotStop(){
    pivotMotor.set(0);
    pivotMotor.set(0);
  }





  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Pivot Position",pivotMotorEncoder.getPosition());
}
}
