// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private XboxController controller;
  private DoubleSupplier leftP;
  private DoubleSupplier rightP;
  private boolean driveStraightToggle = false;
  private double targetAngle;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(DriveTrain subsystem, XboxController gamepad) {
    m_driveTrain = subsystem;
    controller = gamepad;
    leftP = ()->controller.getLeftY();
    rightP = ()->controller.getRightY();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
    // m_driveTrain.resetEncoder(); Do this eventually please
    //I hate science fair
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (((Math.abs(leftP.getAsDouble()) > .5) && (Math.abs(rightP.getAsDouble()) > .5)) && !driveStraightToggle &&
        ((Math.round(leftP.getAsDouble()) + Math.round(rightP.getAsDouble()) == 0))) { // This long if statement checks
                                                                                       // if the sticks are pressed and
                                                                                       // sets the angle to
                                                                                       // automatically adjust to
      targetAngle = m_driveTrain.getAngle();
      driveStraightToggle = true;
    } else if ((Math.abs(leftP.getAsDouble()) < .5) || (Math.abs(rightP.getAsDouble()) < .5)) {// If sticks are let go
      driveStraightToggle = false;
    }
    if ((driveStraightToggle) && (leftP.getAsDouble() > 0)) {// Now this is auto correct with driving forward
      m_driveTrain.setPower((leftP.getAsDouble() + ((targetAngle - m_driveTrain.getAngle()) / 90)),
          rightP.getAsDouble() + ((targetAngle - m_driveTrain.getAngle()) / 90));

    } else if ((driveStraightToggle) && (leftP.getAsDouble() < 0)) {// Now this is auto correct with driving backward
      m_driveTrain.setPower((leftP.getAsDouble() / 2 + ((targetAngle - m_driveTrain.getAngle()) / 90)),
          (rightP.getAsDouble() / 2) + ((targetAngle - m_driveTrain.getAngle()) / 90));
    } else if (((Math.abs(leftP.getAsDouble()) > .2) || (Math.abs((rightP.getAsDouble())) > .2))
        && !driveStraightToggle) { // Cubed control if none of the above conditions apply but the sticks are
                                   // pressed above a threshold
      m_driveTrain.setPower(Math.pow((leftP.getAsDouble()), 3), Math.pow((rightP.getAsDouble()), 3));
    } else {// Obligitory turning it off
      m_driveTrain.setPower(0, 0);
    }

    SmartDashboard.putNumber("Left Power", leftP.getAsDouble());
    SmartDashboard.putNumber("Right Power", rightP.getAsDouble());
    SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
    SmartDashboard.putBoolean("Drive Straight", driveStraightToggle);

    
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
