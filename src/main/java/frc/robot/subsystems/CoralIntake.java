// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final SparkMax wheelMotor = new SparkMax(7, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(2);
    private double speed = 0.3;

    public CoralIntake() {

    }
        

    public void collect() {
        wheelMotor.set(speed);
        // if (limitSwitch.get()) {
        //     wheelMotor.set(0.0);
        // } else {
        //     wheelMotor.set(speed);
        // }
    }

    public void inSoftly() {
        wheelMotor.set(0.05);
    }

    public void score() {
        wheelMotor.set(-speed);
    }

    public boolean hasCoral() {
        return limitSwitch.get();
    }

    public void stop() {
        wheelMotor.set(0.0);
    }

    @Override
    public void periodic() {

    }
}
