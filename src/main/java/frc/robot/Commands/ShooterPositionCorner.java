package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPivot;

public class ShooterPositionCorner extends Command{
    public final ShooterPivot shooterPivot;
    public ShooterPositionCorner(ShooterPivot s){
        this.shooterPivot = s;
    }
    public void execute() {
        shooterPivot.shooterPositionCorner();
      }
    
      // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    shooterPivot.ShooterPivotStop();
    }
}