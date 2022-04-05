// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickClimber extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_Climber;
  private XboxController controller;
  private XboxController controller2;
  private boolean climbToggle = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickClimber(Climber subsystem, XboxController gamepad, XboxController gamepad2) {
    m_Climber = subsystem;
    controller = gamepad;
    controller2 = gamepad2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_Climber.setNeutralMode(); //Turn the climbing motors to brakemode
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller2.getXButtonPressed()){ //This if statement is useless remove it after competition
      climbToggle = !climbToggle;
    }
    SmartDashboard.putBoolean("Cimber's Toggle ", climbToggle);
    

      if(!m_Climber.touch(2)){ //If any touchsensor is pressed, reset the motor encoders
        m_Climber.resetEncoder(2);  //2 is RightWinch
      }
      if(!m_Climber.touch(3)){     //3 is LeftWinch
        m_Climber.resetEncoder(3);
      }
      if(!m_Climber.touch(0)){    //0 is Right Reacher
        m_Climber.resetEncoder(0);
      }
      if(!m_Climber.touch(1)){    //1 is Left Reacher
        m_Climber.resetEncoder(1);
      }
      

        if (Math.abs(controller2.getRightY()) > .2) { //Control Right Winch with right stick
          if(!m_Climber.touch(2)){ //Don't let the motor go past the touch sensor
            m_Climber.setPowerRightWinch(Math.min(controller2.getRightY(), 0));
          }else if(m_Climber.getRightWinchEncoder()<-236000){//Set the high limit to the winch
            m_Climber.setPowerRightWinch(Math.max(controller2.getRightY(), 0));
          }else{//Normal control if above conditions are not met
            m_Climber.setPowerRightWinch(controller2.getRightY());
          }
        } else {
            m_Climber.setPowerRightWinch(0);
        }
// at this point we are going to build tetris in the smart dashboard for gabriel to play when he gets bored-L;
      if (Math.abs(controller2.getLeftY()) > .2) {
        if(!m_Climber.touch(3)){
          m_Climber.setPowerLeftWinch(Math.max(-controller2.getLeftY(), 0));
        }else if(m_Climber.getLeftWinchEncoder()>228000){
          m_Climber.setPowerLeftWinch(Math.min(-controller2.getLeftY(), 0));
        }else{
          m_Climber.setPowerLeftWinch(-controller2.getLeftY());
        }
      } else {
          m_Climber.setPowerLeftWinch(0);
      }

      if (Math.abs(controller2.getRightTriggerAxis()) > .2) {
        if(m_Climber.getRightReacherEncoder()>1.25){//Was 4.5
          m_Climber.setRightReacher(0);
        }else{
          m_Climber.setRightReacher(controller2.getRightTriggerAxis()/4);
        }
      }else if(Math.abs(controller2.getLeftTriggerAxis()) > .2){
        if(!m_Climber.touch(0)){
          m_Climber.setRightReacher(0);
        }else{
          m_Climber.setRightReacher(-controller2.getLeftTriggerAxis()/4);
        }
      }else{
        m_Climber.setRightReacher(0);
      }

      if (controller2.getRightBumper()) {
        if(m_Climber.getLeftReacherEncoder()<-1.25){
          m_Climber.setLeftReacher(0);
        }else{
          m_Climber.setLeftReacher(.75);
        }
      }else if(controller2.getLeftBumper()){
        if(!m_Climber.touch(1)){
          m_Climber.setLeftReacher(0);
        }else{
          m_Climber.setLeftReacher(-1);
        }
      }else{
        m_Climber.setLeftReacher(0);
      }

      
      
      // if (Math.abs(controller2.getLeftY()) > .2) {
      //   if(!m_Climber.touch(1)){
      //     m_Climber.setLeftReacher(Math.min(controller2.getLeftY(), 0));
      //   }else if(m_Climber.getLeftReacherEncoder()<-5){
      //     m_Climber.setLeftReacher(Math.max(controller2.getLeftY(), 0));
      //   }else{
      //     m_Climber.setLeftReacher(controller2.getLeftY());
      //   }
      // } else {
      //     m_Climber.setLeftReacher(0);
      // }

      

    SmartDashboard.putBoolean("Touch BR", m_Climber.touch(0));
    SmartDashboard.putBoolean("Touch BL", m_Climber.touch(1));
    SmartDashboard.putBoolean("Touch TR", m_Climber.touch(2));
    SmartDashboard.putBoolean("Touch TL", m_Climber.touch(3));
    SmartDashboard.putNumber("Right Stick", controller2.getRightY());
    SmartDashboard.putNumber("Left winch Encoder: ", m_Climber.getLeftWinchEncoder());
    SmartDashboard.putNumber("Right winch Encoder: ", m_Climber.getRightWinchEncoder());
    SmartDashboard.putNumber("Left reacher Encoder: ", m_Climber.getLeftReacherEncoder());
    SmartDashboard.putNumber("Right reacher Encoder: ", m_Climber.getRightReacherEncoder());
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