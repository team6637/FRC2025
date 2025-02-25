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
    private final double maxSetpoint = 210.5;
    private double setpoint = minSetpoint;
    private double kP = 0.034;
    private double upKP = 0.04;
    private boolean usingPID = true;
    private double setpointIncrementer = 1.0;
    private final PIDController pid;
    private double level4Setpoint = 210.5;
    private double level3Setpoint = 18;
    private double level2Setpoint = 67;
    private double receiveSetpoint = 111;
    private double startingSetpoint = 0.0;
    private boolean respectMinimumSetpoint = true;
    private boolean isMovingUp;
    
    public Lift() {

        motorLeft = new SparkMax(4, MotorType.kBrushless);
        motorRightFollower = new SparkMax(5, MotorType.kBrushless);
        pid = new PIDController(kP, 0.0, 0.0);
        pid.setTolerance(10);

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

    public void goToLevel4() {
        if (usingPID) {
            setSetpoint(level4Setpoint);
        }
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

    public void goToStart() {
        if (usingPID) {
            setSetpoint(startingSetpoint);
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
        isMovingUp = true;
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
        isMovingUp = false;
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
        isMovingUp = (newSetpoint > setpoint) ? true : false;

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
        //double tempKp = SmartDashboard.getNumber("lift kp", kP);
        double tempKp = isMovingUp ? upKP : kP;
        pid.setPID(tempKp, 0.0, 0.0);

        SmartDashboard.putNumber("lift kp", pid.getP());

        if (usingPID) {
            double speed = pid.calculate(getPosition(), setpoint);
            SmartDashboard.putNumber("lift speed", speed);
            SmartDashboard.putNumber("lift error", pid.getError());
            motorLeft.set(speed);
        }
    }
}