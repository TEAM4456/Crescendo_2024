package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Elevator extends SubsystemBase {
    private CANSparkMax elevatorRight;
    private RelativeEncoder elevatorRightEncoder;
    private final SparkPIDController elevatorRightPIDController;

    private CANSparkMax elevatorLeft;
    private RelativeEncoder elevatorLeftEncoder;
    private final SparkPIDController elevatorLeftPIDController;

  public Elevator() {
    elevatorRight = new CANSparkMax(18,MotorType.kBrushless);
    elevatorRight.setOpenLoopRampRate(.5);
    elevatorRightEncoder = elevatorRight.getEncoder();
    elevatorRightPIDController = elevatorRight.getPIDController();

    elevatorLeft = new CANSparkMax(17,MotorType.kBrushless);
    elevatorLeft.setOpenLoopRampRate(.5);
    elevatorLeftEncoder = elevatorLeft.getEncoder();
    elevatorLeftPIDController = elevatorLeft.getPIDController();

    elevatorRightPIDController.setP(1);
    elevatorRightPIDController.setP(0);
    elevatorRightPIDController.setP(0);
    elevatorRightPIDController.setP(0);

    elevatorLeftPIDController.setP(1);
    elevatorLeftPIDController.setP(0);
    elevatorLeftPIDController.setP(0);
    elevatorLeftPIDController.setP(0);

  }
    
  public void setElevatorPositionUp(){
    elevatorLeftPIDController.setReference(Constants.ElevatorPositions.leftElevatorUp, CANSparkMax.ControlType.kPosition);
    elevatorRightPIDController.setReference(Constants.ElevatorPositions.rightElevatorUp, CANSparkMax.ControlType.kPosition);
  }
  public void setElevatorPositionDown(){
    elevatorLeftPIDController.setReference(Constants.ElevatorPositions.leftElevatorDown, CANSparkMax.ControlType.kPosition);
    elevatorRightPIDController.setReference(Constants.ElevatorPositions.rightElevatorDown, CANSparkMax.ControlType.kPosition);
  }
  public Command setElevatorPositionUpCommand(){
    return run(() -> setElevatorPositionUp()).until(() -> (Math.abs(elevatorRightEncoder.getPosition() - Constants.ElevatorPositions.rightElevatorUp) < 1) && (Math.abs(elevatorLeftEncoder.getPosition() - Constants.ElevatorPositions.leftElevatorUp) < 1));
  }
  public Command setElevatorPositionDownCommand(){
    return run(() -> setElevatorPositionDown()).until(() -> (Math.abs(elevatorRightEncoder.getPosition() - Constants.ElevatorPositions.rightElevatorDown) < 1) && (Math.abs(elevatorLeftEncoder.getPosition() - Constants.ElevatorPositions.leftElevatorDown) < 1));
  }
  @Override
  public void periodic() {

}
}
