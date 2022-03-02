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
    private DigitalInput bottomLimit = new DigitalInput(9);

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
    public boolean getTouch(){
        return bottomLimit.get();
    }

    public void setReachers(double power){
        leftReacher.set(power);
        rightReacher.set(power);
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
