package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public Climber(){}
    private TalonFX leftWinch = new TalonFX(9);
    private TalonFX rightWinch = new TalonFX(10);
    private CANSparkMax leftReacher = new CANSparkMax(12,MotorType.kBrushless);
    private CANSparkMax rightReacher = new CANSparkMax(11,MotorType.kBrushless);
    private DigitalInput bottomRightTouch = new DigitalInput(2);
    private DigitalInput bottomLeftTouch = new DigitalInput(3);
    private DigitalInput topRightTouch = new DigitalInput(0);
    private DigitalInput topLeftTouch = new DigitalInput(1);

    public void setPowerRightWinch(double rightP){
        rightWinch.set(ControlMode.PercentOutput, rightP);
    }
    public void setPowerLeftWinch(double leftP){
        leftWinch.set(ControlMode.PercentOutput, leftP);
    }

    public void setLeftReacher(double leftP){
        leftReacher.set(leftP);
    }

    public void setRightReacher(double rightP){
        rightReacher.set(rightP);
    }

    public void setNeutralMode(){
        leftWinch.setNeutralMode(NeutralMode.Brake);
        rightWinch.setNeutralMode(NeutralMode.Brake);
        leftReacher.setIdleMode(IdleMode.kBrake);
        rightReacher.setIdleMode(IdleMode.kBrake);
    }

    public double getLeftEncoder(){
        return leftWinch.getSelectedSensorPosition();
    }
    public double getRightEncoder(){
        return rightWinch.getSelectedSensorPosition();
    }

    public void resetEncoder(int i){
        if(i==0){
            
          }else if(i==1){
            
          }else if(i==2){
            rightWinch.setSelectedSensorPosition(0);
          }else{
            leftWinch.setSelectedSensorPosition(0);
          }
    }

    public boolean touch(int i){
        if(i==0){
          return bottomRightTouch.get();
        }else if(i==1){
          return bottomLeftTouch.get();
        }else if(i==2){
          return topRightTouch.get();
        }else{
          return topLeftTouch.get();
        }
        
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
