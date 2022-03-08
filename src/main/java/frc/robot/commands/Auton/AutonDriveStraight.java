// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonDriveStraight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private double distance;
  private Timer timer = new Timer();
  


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonDriveStraight(DriveTrain subsystem, double inches) {
    m_driveTrain = subsystem;
    distance = inches;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoder();;  
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((distance-m_driveTrain.getEncoderCount())>500){
      m_driveTrain.setPower(-.2,-.2);
    }else if((distance-m_driveTrain.getEncoderCount())<-500){
      m_driveTrain.setPower(.075,.075);
    } 
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("Encoder", m_driveTrain.getEncoderCount());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setPower(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if(timer.get()<2.5){
    return false;
  }else{
    return true;
  }
}
}
