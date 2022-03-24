// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Shooter() {}
  private TalonFX leftShooter = new TalonFX(6);
  private TalonFX rightShooter = new TalonFX(5);
  private VictorSPX intake = new VictorSPX(7);
  private VictorSPX index = new VictorSPX(8);
  //private ColorSensorV3 cdSeneor = new ColorSensorV3(I2C.Port.kOnboard);
  

  public void setPowerShooter(double power){
    leftShooter.set(ControlMode.Velocity, power);
    rightShooter.set(ControlMode.Follower, 6);
  }

  public void stopShooters(){
    leftShooter.set(ControlMode.PercentOutput, 0);
    rightShooter.set(ControlMode.PercentOutput, 0);
  }

  public void configShooterPID(){
    leftShooter.config_kF(0, 0.06);
    leftShooter.config_kP(0, 0.16);
    leftShooter.config_kD(0, 10);
    rightShooter.setInverted(InvertType.OpposeMaster);
  }

  public void setPowerIntake(DoubleSupplier power){
    intake.set(ControlMode.PercentOutput, power.getAsDouble());
  }
  
  public void setPowerIndex(DoubleSupplier power){
    index.set(ControlMode.PercentOutput, power.getAsDouble());
    index.setNeutralMode(NeutralMode.Brake);
  }

  // public int getDistance(){
  //   return cdSeneor.getProximity();
  // }

  public double getLeftVelocity(){
   return leftShooter.getSelectedSensorVelocity();     
  }

  public double getRightVelocity(){
    return rightShooter.getSelectedSensorVelocity();     
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
