package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public Climber(){}
    private TalonFX leftWinch = new TalonFX(9);
    private TalonFX rightWinch = new TalonFX(10);

    public void setPowerRightWinch(double rightP){
        rightWinch.set(ControlMode.PercentOutput, rightP);
    }
    public void setPowerLeftWinch(double leftP){
        leftWinch.set(ControlMode.PercentOutput, leftP);
    }

    public void setNeutralMode(){
        leftWinch.setNeutralMode(NeutralMode.Brake);
        rightWinch.setNeutralMode(NeutralMode.Brake);
    }

    public double getLeftEncoder(){
        return leftWinch.getSelectedSensorPosition();
    }
    public double getRightEncoder(){
        return rightWinch.getSelectedSensorPosition();
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
