// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



  public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {} 
    private TalonFX fl = new TalonFX(2);
    private TalonFX fr = new TalonFX(1);
    private TalonFX bl = new TalonFX(4);
    private TalonFX br = new TalonFX(3);
    private Servo vision = new Servo(0);
    private AHRS gyro = new AHRS(Port.kMXP);


  public void setPower(double leftP, double rightP){
    fl.set(ControlMode.PercentOutput, -leftP);
    fr.set(ControlMode.PercentOutput, rightP);
    bl.set(ControlMode.PercentOutput, -leftP);
    br.set(ControlMode.PercentOutput, rightP);
  }

  public void coastMode(){
    fl.setNeutralMode(NeutralMode.Coast);
    fr.setNeutralMode(NeutralMode.Coast);
    bl.setNeutralMode(NeutralMode.Coast);
    br.setNeutralMode(NeutralMode.Coast);
  }

  public double getAngle(){
    return gyro.getAngle();
  }

  public void setCurrentLimit(){
    //fl.configOpenloopRamp(.2);
    //fr.configOpenloopRamp(.2);
    //bl.configOpenloopRamp(.2);
    //br.configOpenloopRamp(.2);
    fl.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45,50,1));
    fr.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45,50,1));
    bl.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45,50,1));
    br.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45,50,1));
  }

  public void setCurrentLimitZero(){
    fl.configOpenloopRamp(0);
    fr.configOpenloopRamp(0);
    bl.configOpenloopRamp(0);
    br.configOpenloopRamp(0);
  }

  public double getEncoderCount(){
    return fl.getSelectedSensorPosition();
  }

  public void setServo(double pos){
    vision.set(pos);
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void resetEncoder(){
    fl.setSelectedSensorPosition(0);
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
