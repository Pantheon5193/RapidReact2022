// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JoystickClimber;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.JoystickShooter;
import frc.robot.commands.Auton.AutonAdjust;
import frc.robot.commands.Auton.AutonDriveStraight;
import frc.robot.commands.Auton.AutonIndex;
import frc.robot.commands.Auton.AutonIntake;
import frc.robot.commands.Auton.AutonShooter;
import frc.robot.commands.Auton.AutonShooterStop;
import frc.robot.commands.Auton.AutonTurn;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final JoystickShooter joystickShooter = new JoystickShooter(shooter,controller, controller2);
  private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain, controller, controller2);
  private final JoystickClimber joystickClimber = new JoystickClimber(climber, controller, controller2);
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
        new ParallelCommandGroup(new AutonDriveStraight(driveTrain, 50000), new AutonIntake(shooter, 1)),
        new AutonIntake(shooter, 0),
        new AutonTurn(driveTrain, 200),
        //new AutonDriveStraight(driveTrain, 33000),
        new ParallelCommandGroup(new AutonAdjust(driveTrain), new AutonShooter(shooter,14000)),
        new AutonIndex(shooter, -.5, .5),
        new AutonIndex(shooter, 0, 1),
        new AutonIndex(shooter, -.5, 1),
        new AutonIndex(shooter, 0, .1),
        new AutonShooterStop(shooter)
        );
  }

  public Command getDriveCommand(){
    return joystickDrive;
    //return null;
  }
  public Command getShooterCommand(){
    return joystickShooter;
    //return null;
  }
  public Command getClimberCommand(){
    return joystickClimber;
    //return null;
  }
}
