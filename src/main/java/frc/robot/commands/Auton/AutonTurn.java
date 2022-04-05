// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.ExampleSubsystem;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonTurn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private double angle;
  private Timer timer = new Timer();
  double P = 0.003;
  double I = 0;//0.001;
  double D = 0;//0.0003;
  double error = 0;
  double pError = 0;
  double integral =0;
  


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonTurn(DriveTrain subsystem, double degree) {
    m_driveTrain = subsystem;
    angle = degree;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();  
    timer.reset();
    timer.start();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      pError = error;
      error = angle - m_driveTrain.getAngle();
      integral += (error*.02);
      final double power = P*error + I*integral + D*((error-pError)/.02);
      m_driveTrain.setPower(power,-power);  
      SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
      SmartDashboard.putNumber("Gyro Error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setPower(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if(timer.get()<3){
    return false;
  }else{
    return true;
  }
}
}
