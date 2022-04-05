// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  private final DriveTrain m_driveTrain;
  private XboxController controller;
  private XboxController controller2;
  private DoubleSupplier leftP;
  private DoubleSupplier rightP;
  private boolean driveStraightToggle = false;
  private boolean climbToggle = false;
  private double targetAngle;
  private double averagePow;
  private double angleToGoalDeg;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(DriveTrain subsystem, XboxController gamepad, XboxController gamepad2) {
    m_driveTrain = subsystem;
    controller = gamepad;
    controller2 = gamepad2;
    leftP = ()->controller.getLeftY();
    rightP = ()->controller.getRightY();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
    m_driveTrain.coastMode();
    m_driveTrain.resetEncoder();
    m_driveTrain.setCurrentLimit();
    // m_driveTrain.resetEncoder(); Do this eventually please
    //I hate science fair
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller2.getBButtonPressed()){
      m_driveTrain.setCurrentLimit();
    }
    if(controller2.getAButtonPressed()){
      m_driveTrain.setCurrentLimitZero();
    }

    if(!climbToggle){
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);
      
      averagePow= (Math.pow(leftP.getAsDouble(), 3)+Math.pow(rightP.getAsDouble(), 3))/2;
      if (((Math.abs(leftP.getAsDouble()) > .5) && (Math.abs(rightP.getAsDouble()) > .5)) && !driveStraightToggle &&
          ((Math.abs(Math.round(leftP.getAsDouble()) + Math.round(rightP.getAsDouble())) == 2))) { // This long if statement checks
                                                                                         // if the sticks are pressed and
                                                                                         // sets the angle to
                                                                                         // automatically adjust to
        targetAngle = m_driveTrain.getAngle();
        driveStraightToggle = true;
      } else if ((Math.abs(leftP.getAsDouble()) < .5) || (Math.abs(rightP.getAsDouble()) < .5)) {// If sticks are let go
        driveStraightToggle = false;
      }
      if ((driveStraightToggle) && (leftP.getAsDouble() > 0)) {// Now this is auto correct with driving backward
        m_driveTrain.setPower(averagePow - ((targetAngle - m_driveTrain.getAngle()) / 90),
            averagePow - ((targetAngle - m_driveTrain.getAngle()) / 90));
  
      } else if ((driveStraightToggle) && (leftP.getAsDouble() < 0)) {// Now this is auto correct with driving forward
        m_driveTrain.setPower(averagePow + ((targetAngle - m_driveTrain.getAngle()) / 90),
            (averagePow + ((targetAngle - m_driveTrain.getAngle()) / 90)));
      } else if (((Math.abs(leftP.getAsDouble()) > .2) || (Math.abs((rightP.getAsDouble())) > .2))
          && !driveStraightToggle) { // Cubed control if none of the above conditions apply but the sticks are
                                     // pressed above a threshold
        m_driveTrain.setPower(Math.pow((leftP.getAsDouble()), 3), Math.pow((rightP.getAsDouble()), 3));
      }else if(controller.getYButtonPressed()){
        m_driveTrain.resetGyro();
      }else if(controller.getYButton()){
        if(area!=0){
          m_driveTrain.setPower(-x/ 60, x/60);
        }
      } else {// Obligitory turning it off
        m_driveTrain.setPower(0, 0);
      }
      if(controller.getYButton()){
        m_driveTrain.setServo(.85); //Low Value is .
        table.getEntry("pipeline").setNumber(0);
        //SmartDashboard.putNumber("Distance to Goal", (104-31)/(Math.tan(Math.toRadians(angleToGoalDeg))));
      }else{
        m_driveTrain.setServo(.575);
        table.getEntry("pipeline").setNumber(1);
      }

      if(controller.getXButton()){
        if(area!=0){
          m_driveTrain.setPower(-x/ 75, x/75);
        }
        if(leftP.getAsDouble()<-.2){
          averagePow = Math.pow((leftP.getAsDouble()), 3);
          m_driveTrain.setPower(averagePow + ((-x) / 75),
            (averagePow + ((x) / 75)));
        }

      }
  
      
      angleToGoalDeg = 35 +y;
      SmartDashboard.putNumber("Encoder", m_driveTrain.getEncoderCount());
      SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
      SmartDashboard.putBoolean("Drive Toggle", driveStraightToggle);
      SmartDashboard.putNumber("TargetAngle", targetAngle);
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.coastMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
