// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Shooter() {}
  private TalonFX leftShooter = new TalonFX(6);
  private TalonFX rightShooter = new TalonFX(5);
  private VictorSPX intake = new VictorSPX(7);
  private VictorSPX index = new VictorSPX(8);

  public void setPowerShooter(DoubleSupplier power){
    leftShooter.set(ControlMode.PercentOutput, power.getAsDouble());
    //rightShooter.set(ControlMode.PercentOutput, -power.getAsDouble());
  }

  public void setPowerIntake(DoubleSupplier power){
    intake.set(ControlMode.PercentOutput, power.getAsDouble());
  }
  
  public void setPowerIndex(DoubleSupplier power){
    index.set(ControlMode.PercentOutput, power.getAsDouble());
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
