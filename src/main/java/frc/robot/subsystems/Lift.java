// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
    private final SparkMax motorLeft;
    private final SparkMax motorRightFollower;
    private final double minSetpoint = 0.0;
    private final double maxSetpoint = 214;
    private double setpoint = minSetpoint;
    private double kP = 0.025;
    private boolean usingPID = true;
    private double setpointIncrementer = 1.0;
    private final PIDController pid;
    private double level3Setpoint = 214.0;
    private double level2Setpoint = 87;
    private double level1Setpoint = 0;
    private double receiveSetpoint = 0;
    private boolean respectMinimumSetpoint = true;
    
    public Lift() {

        motorLeft = new SparkMax(4, MotorType.kBrushless);
        motorRightFollower = new SparkMax(5, MotorType.kBrushless);
        pid = new PIDController(kP, 0.0, 0.0);

        SmartDashboard.putNumber("lift kp", kP);

        
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig motorLeftConfig = new SparkMaxConfig();
        SparkMaxConfig motorRightFollowerConfig = new SparkMaxConfig();

        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);

        motorLeftConfig
            .apply(globalConfig)
            .inverted(true);

        motorRightFollowerConfig
            .apply(globalConfig)
            .follow(motorLeft, true);

        motorLeft.configure(motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorRightFollower.configure(motorRightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void goToLevel3() {
        if (usingPID) {
            setSetpoint(level3Setpoint);
        }
    }

    public void goToLevel2() {
        if (usingPID) {
            setSetpoint(level2Setpoint);
        }
    }

    public void goToLevel1() {
        if (usingPID) {
            setSetpoint(level1Setpoint);
        }
    }

    public void goToRecieveSetpoint() {
        if (usingPID) {
            setSetpoint(receiveSetpoint);
        }
    }

    public double getPosition() {
        return motorLeft.getEncoder().getPosition();
    }

    public void setRespectMinimumSetpoint(boolean v) {
        respectMinimumSetpoint = v;
    }

    public void extend() {
        if (usingPID) {
            setSetpoint(setpoint += setpointIncrementer);
        } else {
            if (getPosition() >= maxSetpoint) {
                motorLeft.set(0.0);
            } else {
                motorLeft.set(0.3);
            } 
        }
    }

    public void retract() {
        if (usingPID) {
            setSetpoint(setpoint -= setpointIncrementer);
         } else {
             if (getPosition() <= minSetpoint) {
                 motorLeft.set(0.0);
             } else {
                 motorLeft.set(-0.3);
             }
         }
     }

     public void stop() {
        if(!usingPID) {
            motorLeft.set(0.0);
        }
    }

    public void setSetpoint(double newSetpoint) {
        if (newSetpoint > maxSetpoint) {
            setpoint = maxSetpoint;
        } else if (newSetpoint < minSetpoint && respectMinimumSetpoint) {
            setpoint = minSetpoint;
        } else {
            setpoint = newSetpoint;
        }
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("lift position", getPosition());
        SmartDashboard.putNumber("lift setpoint", setpoint);

        // Comment out next 2 lines once the robot is tuned
        double tempKp = SmartDashboard.getNumber("lift kp", kP);
        pid.setPID(tempKp, 0.0, 0.0);
        SmartDashboard.putNumber("lift temp kp", tempKp);

        if (usingPID) {
            double speed = pid.calculate(getPosition(), setpoint);
            SmartDashboard.putNumber("lift speed", speed);
            SmartDashboard.putNumber("lift error", pid.getError());
            SmartDashboard.putNumber("lift pid kp", pid.getP());
            motorLeft.set(speed);
        }
    }
}











//Forrest Was Here