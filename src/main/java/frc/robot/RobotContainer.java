package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Commands.DumpOutIntake;
import frc.robot.Commands.ElevatorDown;
import frc.robot.Commands.ElevatorUp;
import frc.robot.Commands.FeedForward;
import frc.robot.Commands.FeedForwardContinuous;
import frc.robot.Commands.FeedIn;
import frc.robot.Commands.HatchOpener;
import frc.robot.Commands.IntakeIn;
import frc.robot.Commands.IntakeInContinuous;
import frc.robot.Commands.MoveIntakeIn;
import frc.robot.Commands.MoveIntakeOut;
import frc.robot.Commands.ShooterAmp;
import frc.robot.Commands.ShooterDown;
import frc.robot.Commands.ShooterIntake;
import frc.robot.Commands.ShooterOn;
import frc.robot.Commands.ShooterOnContinuous;
import frc.robot.Commands.ShooterPositionCenter;
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
  private final CommandXboxController buttonBoard = new CommandXboxController(1);
  private final CommandXboxController backupManual = new CommandXboxController(2);

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


    //Pathplanner Auto Components


    chooser = new SendableChooser<Command>();
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






    //Speaker Sequences
    public Command SpeakerCenterSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionCenterCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Speaker Front")),
           new InstantCommand(()->shooter.shooterOn())
           ),
          new FeedForwardContinuous(shooter,intake)
      );
    }

    public Command SpeakerRightSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionCornerCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Speaker Left")),
           new InstantCommand(()->shooter.shooterOn())
           ),
          new FeedForwardContinuous(shooter,intake)
      );
    }

    public Command SpeakerLeftSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionCornerCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Speaker Right")),
           new InstantCommand(()->shooter.shooterOn())
           ),
          new FeedForwardContinuous(shooter,intake)
      );
    }


    //Amp Sequence
    public Command AmpScoreSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionAmpCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Amp Score")),
           new InstantCommand(()->shooter.shooterAmp())
           ),
          new FeedForwardContinuous(shooter,intake)
      );
    }

   
    //Source Sequences
    public Command SourceFarSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionSourceCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Source Far")),
           new InstantCommand(()->shooter.shooterIntake())
           )
      );
    }
    public Command SourceMidSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionSourceCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Source Middle")),
           new InstantCommand(()->shooter.shooterIntake())
           )
      );
    }
     public Command SourceCloseSequence(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           shooterPivot.shooterPositionSourceCommand(),
           AutoBuilder.followPath(PathPlannerPath.fromPathFile("Source Close")),
           new InstantCommand(()->shooter.shooterIntake())
           )
      );
    }
    //Auto Sequences Pieces Center
    public Command autoCenter2Piece(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
        stopMotors(),
        new InstantCommand(()-> intake.speedForward()),
        intakePulley.intakePositionGroundCommand(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 2 Out Center"))
        ),
        new ParallelCommandGroup(
          new InstantCommand(()->shooter.shooterOn()),
          shooterPivot.shooterPositionCenterCommand(),
          intakePulley.intakePositionFeedCommand(),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 2 Back Center"))
        ),
        new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward())
        );
    }
    public Command autoCenter1Piece(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
        stopMotors(),
        new InstantCommand(()-> intake.speedForward()),
        intakePulley.intakePositionGroundCommand(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 1 Out Center"))
        ),
        new ParallelCommandGroup(
          new InstantCommand(()->shooter.shooterOn()),
          shooterPivot.shooterPositionCenterCommand(),
          intakePulley.intakePositionFeedCommand(),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 1 Back Center"))
        ),
        new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward())
        );
    }
    public Command autoCenter3Piece(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
        stopMotors(),
        new InstantCommand(()-> intake.speedForward()),
        intakePulley.intakePositionGroundCommand(),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 3 Out Center"))
        ),
        new ParallelCommandGroup(
          new InstantCommand(()->shooter.shooterOn()),
          shooterPivot.shooterPositionCenterCommand(),
          intakePulley.intakePositionFeedCommand(),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 3 Back Center"))
        ),
        new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward())
        );
    }

    //center Autos
    public Command autoCenter2(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCenterCommand(),
          new InstantCommand(()->shooter.shooterOn()),
          intakePulley.intakePositionGroundCommand()
      ),
      new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward()),
      new WaitCommand(1),
      autoCenter2Piece()
    );}

    public Command autoCenter12(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCenterCommand(),
          new InstantCommand(()->shooter.shooterOn()),
          intakePulley.intakePositionGroundCommand()
      ),
      new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward()),
      new WaitCommand(.5),
      autoCenter2Piece(),
      new WaitCommand(1),
      autoCenter1Piece()
    );}
    
    public Command autoCenter23(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCenterCommand(),
          new InstantCommand(()->shooter.shooterOn()),
          intakePulley.intakePositionGroundCommand()
      ),
      new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward()),
      new WaitCommand(.5),
      autoCenter2Piece(),
      new WaitCommand(1),
      autoCenter3Piece()
    );}

    public Command autoCenter123(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCenterCommand(),
          new InstantCommand(()->shooter.shooterOn()),
          intakePulley.intakePositionGroundCommand()
      ),
      new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward()),
      new WaitCommand(.5),
      autoCenter2Piece(),
      new WaitCommand(1),
      autoCenter1Piece(),
      new WaitCommand(1),
      autoCenter3Piece()
    );}

    //Amp Side Auto 
    public Command autoAmpSide1(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCornerCommand(),
          new InstantCommand(()->shooter.shooterOn()),
          intakePulley.intakePositionGroundCommand()
      ),
      new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward()),
      new WaitCommand(1),
      new ParallelCommandGroup(
        stopMotors(),
        new InstantCommand(()-> intake.speedForward()),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 1 Out Amp"))
        ),
        new ParallelCommandGroup(
          new InstantCommand(()->shooter.shooterOn()),
          shooterPivot.shooterPositionCornerCommand(),
          intakePulley.intakePositionFeedCommand(),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 1 Back Amp"))
        ),
        new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward())
      );
    }

    //Source Side Auto
     public Command autoSourceSide3(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCornerCommand(),
          new InstantCommand(()->shooter.shooterOn()),
          intakePulley.intakePositionGroundCommand()
      ),
      new InstantCommand(()-> shooter.feedForward()),
      new InstantCommand(() -> intake.intakeFeedForward()),
      new InstantCommand(()-> intake.speedForward()),
      new WaitCommand(.5),
      new ParallelCommandGroup(
        stopMotors(),
        new InstantCommand(()-> intake.speedForward()),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 3 Out Source"))
        ),
        new ParallelCommandGroup(
          new InstantCommand(()->shooter.shooterOn()),
          shooterPivot.shooterPositionCornerCommand(),
          intakePulley.intakePositionFeedCommand(),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("Piece 3 Back Source"))
        ),
        new InstantCommand(()-> shooter.feedForward()),
        new InstantCommand(() -> intake.intakeFeedForward()),
        new InstantCommand(()-> intake.speedForward())
      );
    }
    public Command autoSideShootNothing(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCornerCommand(),
          new InstantCommand(()->shooter.shooterOn())
      ),
      new InstantCommand(()-> shooter.feedForward())
      );
    }
    public Command autoCenterShootNothing(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterPivot.shooterPositionCenterCommand(),
          new InstantCommand(()->shooter.shooterOn())
      ),
      new InstantCommand(()-> shooter.feedForward())
      );
    }
    

    //stop Feed and Shoot Motors
    public Command stopMotors(){
      return new SequentialCommandGroup(
        new InstantCommand(()-> shooter.feedStop()),
        new InstantCommand(()-> shooter.shooterOff()),
        new InstantCommand(()-> intake.intakeFeedStop())
      );
    }
     public Command stopMotorsAll(){
      return new SequentialCommandGroup(
        new InstantCommand(()-> shooter.feedStop()),
        new InstantCommand(()-> shooter.shooterOff()),
        new InstantCommand(()-> intake.intakeFeedStop()),
        new InstantCommand(()-> intake.speedStop())
      );
    }
    public Command ShootCenter(){
      return new SequentialCommandGroup(
        new ParallelCommandGroup(
        shooterPivot.shooterPositionCenterCommand(),
        new InstantCommand(()->shooter.shooterOn())),
        new InstantCommand(()->shooter.feedForward()));
        
    }


  
  private void configureButtonBindings() {
    chooser.setDefaultOption("nothing", null);

    chooser.addOption("Center 1-2",autoCenter12());
    chooser.addOption("Center 2",autoCenter2());
    chooser.addOption("Center 2-3",autoCenter23());
    chooser.addOption("Amp Side 1",autoAmpSide1());
    chooser.addOption("Source Side 3",autoSourceSide3());
    chooser.addOption("Side Shoot Nothing",autoSideShootNothing());
    chooser.addOption("Center Shoot Nothing", autoCenterShootNothing());
    chooser.addOption("Center 1-2-3",autoCenter123());
    /* Driver Buttons */
    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> driver.getRawAxis(rotationAxis)));
 
    driver.leftTrigger().whileTrue(elevator.setElevatorPositionDownCommand());
    driver.rightTrigger().whileTrue(elevator.setElevatorPositionUpCommand());

    driver.rightBumper().whileTrue(new ElevatorUp(elevator));
    driver.leftBumper().whileTrue(new ElevatorDown(elevator));


    driver.b().whileTrue(shooterPivot.shooterPositionUpCommand());
    driver.x().whileTrue(shooterPivot.shooterPositionDownCommand());
    
    driver.y().whileTrue(new ShooterUp(shooterPivot));
    driver.a
    ().whileTrue(new ShooterDown(shooterPivot));

    driver.start().whileTrue(stopMotorsAll());

    buttonBoard.a().whileTrue(SourceMidSequence()); 
    buttonBoard.b().whileTrue(SourceCloseSequence());
    buttonBoard.leftTrigger().whileTrue(SourceFarSequence());

    buttonBoard.leftBumper().whileTrue(SpeakerLeftSequence());
    buttonBoard.x().whileTrue(SpeakerCenterSequence());
    buttonBoard.y().whileTrue(SpeakerRightSequence());

    buttonBoard.rightBumper().whileTrue(new FeedIn(shooter, intake));
    buttonBoard.rightTrigger().whileTrue(ShootCenter());


    backupManual.y().whileTrue(new ShooterOn(shooter));
    backupManual.a().whileTrue(new FeedForward(shooter, intake));
    backupManual.b().whileTrue(shooterPivot.shooterPositionCenterCommand());

    backupManual.rightTrigger().whileTrue(intakePulley.intakePositionGroundCommand());
    backupManual.leftTrigger().whileTrue(intakePulley.intakePositionFeedCommand());
    backupManual.leftBumper().whileTrue(new IntakeIn(intake));
    backupManual.start().whileTrue(stopMotorsAll());
    backupManual.back().whileTrue(new DumpOutIntake(shooter, intake));

    backupManual.x().whileTrue(new ShooterIntake(shooter));







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
