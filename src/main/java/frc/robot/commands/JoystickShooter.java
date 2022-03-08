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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private Timer timer = new Timer();
  private Timer timeoutTimer = new Timer();
  private XboxController controller;
  private boolean touchytouch = false;
  private boolean shooterToggle = false;
  private int ballCount =0;
  private int velocity = 9000;
  private boolean shooterReady = false;
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
    m_shooter.configShooterPID();
    ballCount=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getLeftTriggerAxis()>.5){
      m_shooter.setPowerShooter(velocity);
    }else{
      m_shooter.stopShooters();
    }

    if(controller.getPOV()==90 && timeoutTimer.get() ==0){
      velocity+=500;
      timeoutTimer.start();
    }else if(controller.getPOV()==270 &&timeoutTimer.get() ==0){
      velocity-=500;
      timeoutTimer.start();
    }

    if(timeoutTimer.get()>.2){
      timeoutTimer.stop();
      timeoutTimer.reset();
    }

    if(controller.getRightTriggerAxis()>.1){
      m_shooter.setPowerIntake(() -> controller.getRightTriggerAxis());
    }else{
      m_shooter.setPowerIntake(() -> 0);
    }
    //Target should be at 3.2777 on x
    //22 - .65
    //60 - .7

    //Old Dat
    //82 - .7
    //
    //106 - .8     
    //130 - .9
    // if(controller.getAButtonPressed()){
    //   shooterToggle = !shooterToggle;
    // }

    if(controller.getBButton()){
      m_shooter.setPowerIntake(() -> -.5);
    }
    // if(shooterToggle){
    //   m_shooter.setPowerIntake(() -> 1);
    // }else{
    //   m_shooter.setPowerIntake(() -> 0);
    // }
    if(sensorUpdate()){
      timer.start();
      ballCount++;
    }
    if(timer.get()>2){
      timer.stop();
      timer.reset();
    }
    if(controller.getRightBumper()){
      m_shooter.setPowerIndex(() ->-.5);
    }else if(timer.get()>0){
      if(ballCount==1 && m_shooter.getDistance()>125){
        m_shooter.setPowerIndex(() ->-.3);
      }else if(ballCount==2 && timer.get()<.1){
        m_shooter.setPowerIndex(() ->-.3);
      }else{
        m_shooter.setPowerIndex(() -> 0);
      }
    }else{
      m_shooter.setPowerIndex(() -> 0);
    }

    if(Math.abs(m_shooter.getLeftVelocity() - velocity) <600){
      shooterReady = true;
    }else{
      shooterReady=false;
    }

    SmartDashboard.putNumber("Distance Sensor", m_shooter.getDistance());
    
    SmartDashboard.putBoolean("Sensor Update", sensorUpdate());
    SmartDashboard.putNumber("BallCount", ballCount);
    SmartDashboard.putNumber("Left Power", m_shooter.getLeftVelocity());
    SmartDashboard.putNumber("Right Power", m_shooter.getRightVelocity());
    SmartDashboard.putNumber("Velocity", velocity);
    SmartDashboard.putBoolean("Shooter Ready", shooterReady);
    //118-16000
    //52-12000
    //74 - 12000
    //low - edge - 7000
    //low - 2ft 9000
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
