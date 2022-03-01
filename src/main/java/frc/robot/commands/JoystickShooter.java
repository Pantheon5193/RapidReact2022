// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private Timer timer = new Timer();
  private XboxController controller;
  private boolean touchytouch = false;
  private boolean shooterToggle = false;
  private int ballCount =0;
  private int i;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickShooter(Shooter subsystem, XboxController gamepad) {
    m_shooter = subsystem;
    controller = gamepad;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballCount=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getLeftTriggerAxis()>.5){
      m_shooter.setPowerShooter(() -> .3);
    }else{
      m_shooter.setPowerShooter(() ->0);
    }
    

    if(controller.getAButtonPressed()){
      shooterToggle = !shooterToggle;
    }

    if(shooterToggle){
      m_shooter.setPowerIntake(() -> 1);
    }else{
      m_shooter.setPowerIntake(() -> 0);
    }
    if(sensorUpdate()){
      timer.start();
      ballCount++;
    }
    if(timer.get()>.25){
      timer.stop();
      timer.reset();
    }
    if(controller.getBButton()){
      m_shooter.setPowerIndex(() ->-.3);
    }else if(timer.get()>0){
      if(ballCount==1 && timer.get()<.25){
        m_shooter.setPowerIndex(() ->-.3);
      }else if(ballCount==2 && timer.get()<.25){
        m_shooter.setPowerIndex(() ->-.3);
      }
    }else{
      m_shooter.setPowerIndex(() -> 0);
    }

    touchytouch = m_shooter.touch();
    SmartDashboard.putNumber("Distance Sensor", m_shooter.getDistance());
    SmartDashboard.putBoolean("TopLeftTouch", touchytouch);
    SmartDashboard.putBoolean("Sensor Update", sensorUpdate());
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("BallCount", ballCount);
    SmartDashboard.putNumber("I",i);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean sensorUpdate(){
    boolean value = false;
    if(m_shooter.getDistance()>125 && i==0){
      i=1;
      value = true;
    }else if(m_shooter.getDistance()<125){
      i=0;
    }
    return value;
  }
}
