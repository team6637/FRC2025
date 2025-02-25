// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

  /** Creates a new Climber. */
public class Climber extends SubsystemBase {
    private final SparkMax motor;

    private double minSetpoint = 0;
    private double maxSetpoint = 0;
    private double setpoint = minSetpoint;
    
    public Climber() {
        motor = new SparkMax(10, MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig motorConfig = new SparkMaxConfig();        
        
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);

        motorConfig
            .apply(globalConfig)
            .inverted(false);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void up() {
        // if position is greater or equal to max, make speed 0
        if (getPosition() < maxSetpoint){
            motor.set(0.5);
        } else {
            motor.set(0.0);
        }        
    }

    public void down() {
        if (getPosition() > minSetpoint){
            motor.set(-0.5);
        } else {
            motor.set(0.0);
        }
    }

    public void stop() {
        motor.set(0.0);
    }



    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber position", getPosition());
    }
}
