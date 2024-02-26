package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePulley;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterPivot;
import frc.robot.Subsystems.Swerve;
import frc.robot.Commands.toggleSpeed;
import frc.robot.Commands.ElevatorDown;
import frc.robot.Commands.ElevatorUp;
import frc.robot.Commands.FeedForward;
import frc.robot.Commands.IntakeIn;
import frc.robot.Commands.MoveIntakeIn;
import frc.robot.Commands.MoveIntakeOut;
import frc.robot.Commands.ShooterAmp;
import frc.robot.Commands.ShooterDown;
import frc.robot.Commands.ShooterIntake;
import frc.robot.Commands.ShooterOn;
import frc.robot.Commands.ShooterUp;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, Commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController second = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value ;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
 // private final JoystickButton zeroGyro =
 //     new JoystickButton(driver, XboxController.Button.kY.value);
 //private final JoystickButton robotCentric =
 //     new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Shooter shooter = new Shooter();
  private final Elevator elevator = new Elevator();
  private final ShooterPivot shooterPivot = new ShooterPivot();
  private final IntakePulley intakePulley = new IntakePulley();
  private final Intake intake = new Intake();

  private final SendableChooser<Command> chooser;


  /** The container for the robot. Contains subsystems, OI devices, and Commands. */
  
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis)));

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);
    // Configure the button bindings
    configureButtonBindings();  
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
       
    /* Driver Buttons */
    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> driver.getRawAxis(rotationAxis)));
 
   

    driver.b().whileTrue(new FeedForward(shooter));
    driver.x().toggleOnTrue(new ShooterOn(shooter));
    driver.y().toggleOnTrue(new ShooterIntake(shooter));
    driver.a().whileTrue(new ShooterAmp(shooter));

    driver.rightBumper().whileTrue(new ElevatorUp(elevator));
    driver.leftBumper().whileTrue(new ElevatorDown(elevator));
    
    driver.leftTrigger().whileTrue(new ShooterUp(shooterPivot));
    driver.rightTrigger().whileTrue(new ShooterDown(shooterPivot));

    driver.start().whileTrue(new ElevatorUp(elevator));
    driver.back().whileTrue(new ElevatorDown(elevator));

    second.rightTrigger().whileTrue(new MoveIntakeIn(intakePulley));
    second.leftTrigger().whileTrue(new MoveIntakeOut(intakePulley));
    second.a().toggleOnTrue(new IntakeIn(intake));


  }
  public Swerve getSwerve(){
    return s_Swerve;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {
    s_Swerve.resetModulesToAbsolute();
    return chooser.getSelected();
    //return null;
  }
}
