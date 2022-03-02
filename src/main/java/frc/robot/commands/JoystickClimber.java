// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickClimber extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_Climber;
  private XboxController controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickClimber(Climber subsystem, XboxController gamepad) {
    m_Climber = subsystem;
    controller = gamepad;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_Climber.setNeutralMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (Math.abs(controller.getRightY()) > .2) {
          m_Climber.setPowerRightWinch(controller.getRightY());
          //m_Climber.setRightReacher(controller.getRightY()/8);
      } else {
          m_Climber.setPowerRightWinch(0);
          m_Climber.setRightReacher(0);
      }

      if (Math.abs(controller.getLeftY()) > .2) {
         m_Climber.setPowerLeftWinch(controller.getLeftY());
         //m_Climber.setLeftReacher(controller.getLeftY()/8);
      } else {
          m_Climber.setPowerLeftWinch(0);
          m_Climber.setLeftReacher(0);
      }
      //m_Climber.setReachers(.8);
    
    SmartDashboard.putBoolean("Bottom Limit", m_Climber.getTouch());
    SmartDashboard.putNumber("Left Encoder: ", m_Climber.getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder: ", m_Climber.getRightEncoder());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}