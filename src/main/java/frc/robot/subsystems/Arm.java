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

    private double kP = 0.008;
    private boolean usingPID = true;

    private double setpointIncrementer = 1.0;
    private final PIDController pid;
    private final DutyCycleEncoder throughboreEncoder;

    private double maxSpeed = 0.6;

    private final double intakeSetpoint = 250.0;
    private final double level2Setpoint = 249.0;
    private final double level3Setpoint = 43.0;
    private final double level4Setpoint = 49.0;
    private final double startingSetpoint = 250.0;
    //if we need to turn around for level 3 the lift height needs to be at 187, angle stays same as level 2 for the arm

    private double setpoint = startingSetpoint;
    private double maxSetpoint = 250.0;
    private double minSetpoint = 24;
    
    public Arm() {
        leftMotor = new SparkMax(1, MotorType.kBrushless);
        rightMotorFollower = new SparkMax(2, MotorType.kBrushless);

        throughboreEncoder = new DutyCycleEncoder(0);
        throughboreEncoder.setInverted(true);
        pid = new PIDController(kP, 0.0, 0.0);
        pid.setTolerance(5);


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
        if(tempVal > 260.0) tempVal = 0 - (360-tempVal);
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
            if (getSensorAsDegrees() >= maxSetpoint) {
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

    public void goToStartingPosition() {
        if (usingPID) {
            setSetpoint(startingSetpoint);
        }
    }

    public void goToLevel2Position() {
        if (usingPID) {
            setSetpoint(level2Setpoint);
        }
    }

    public void goToLevel3Position() {
        if (usingPID) {
            setSetpoint(level3Setpoint);
        }
    }

    public void goToLevel4Position() {
        if (usingPID) {
            setSetpoint(level4Setpoint);
        }
    }

    public void backOff() {
        setSetpoint(setpoint += 20);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm position", getSensorAsDegrees());
        SmartDashboard.putNumber("arm setpoint", setpoint);

        if (usingPID) {
            double speed = pid.calculate(getSensorAsDegrees(), setpoint);
            if(Math.abs(speed) > maxSpeed) speed = maxSpeed * Math.signum(speed);
            leftMotor.set(speed);
        }
        
    }
}





















//Forrest Was here