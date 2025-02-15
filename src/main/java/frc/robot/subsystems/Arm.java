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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotorFollower;

    private final double intakeSetpoint = 320.0;
    private final double scoreSetpoint = 127.0;
    private final double startingSetpoint = 360.0;

    private double kP = 0.0;
    private boolean usingPID = false;

    private double setpointIncrementer = 1.0;
    private final PIDController pid;
    private final DutyCycleEncoder throughboreEncoder;

    private double maxSpeed = 0.4;
    private double setpoint = startingSetpoint;
    private double maxSetpoint = 360;
    private double minSetpoint = 100;
    
    public Arm() {
        leftMotor = new SparkMax(1, MotorType.kBrushless);
        rightMotorFollower = new SparkMax(2, MotorType.kBrushless);

        throughboreEncoder = new DutyCycleEncoder(0);
        pid = new PIDController(kP, 0.0, 0.0);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorFollowerConfig = new SparkMaxConfig();

        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);

        leftMotorConfig
            .apply(globalConfig)
            .inverted(false);

        rightMotorFollowerConfig
            .apply(globalConfig)
            .follow(leftMotor, true);

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotorFollower.configure(rightMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    double tempVal;
    public double getSensorAsDegrees() {
        tempVal = throughboreEncoder.get() * 360.0;

        // if the sensor wraps around and reads past 360, go higher than 360
        if (tempVal < 10) {
            tempVal += 360;
        }
        return tempVal;
    }
    
    public void setSetpoint(double newSetpoint) {
        if (newSetpoint > maxSetpoint) {
            setpoint = maxSetpoint;
        } else if (newSetpoint < minSetpoint) {
            setpoint = minSetpoint;
        } else {
            setpoint = newSetpoint;
        }
    }

    public void moveArmTowardBack() {
        if (usingPID) {
            setSetpoint(setpoint -= setpointIncrementer);
         } else {
             if (getSensorAsDegrees() <= minSetpoint) {
                 leftMotor.set(0.0);
             } else {
                 leftMotor.set(-0.2);
             }
         }
    }
    
    public void moveArmTowardFront() {
        if (usingPID) {
            setSetpoint(setpoint += setpointIncrementer);
        } else {
            // account for if the sensor goes past 360 
            if (getSensorAsDegrees() >= maxSetpoint || getSensorAsDegrees() < 10.0) {
                leftMotor.set(0.0);
            } else {
                leftMotor.set(0.2);
            } 
        }
    }

    public void stop() {
        if(!usingPID) {
            leftMotor.set(0.0);
        }
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public void goToRecievePosition() {
        if (usingPID) {
            setSetpoint(intakeSetpoint);
        }
    }

    public void goToScorePosition() {
        if (usingPID) {
            setSetpoint(scoreSetpoint);
        }
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm position", getSensorAsDegrees());

        if (usingPID) {
            double speed = pid.calculate(leftMotor.getEncoder().getPosition(), setpoint);
            if(Math.abs(speed) > maxSpeed) speed = maxSpeed * Math.signum(speed);
            leftMotor.set(speed);
        }
        
    }
}
