---
description: >-
  This is for the specific parts of Pedro that isn't explained as clearly, so
  this can serve as the summary of what I understand myself.
---

# Tougher quirks of Pedro

## **Ivy (Command-based code structure)**

If you're Chandler, you remember NextFTC so you know this one because it's almost the same thing :cry:. Commands are used to control subsystems, which are code that controls/access parts of the robot at a time (Shooter, Intake, Drivetrain, etc.). These subsystems and commands are going to be used in both your TeleOP and Auto.

{% hint style="info" %}
Make sure your Android Studio has Ivy installed. If you don't or don't know, click [here](https://pedropathing.com/docs/ivy).
{% endhint %}

### Subsystems/Commands

Derived from last year's code and [CodeBlood's](https://github.com/BeepBot99/CodeBloodedDecodeV2/tree/master/src/main/java/org/firstinspires/ftc/teamcode/subsystems) code & structure.

{% tabs %}
{% tab title="Intake.java" %}
```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.*;

@Config
public class Intake {
    private boolean slowMode = false;
    private Mode mode = Mode.OFF;
    public static double fastPower = -1;
    public static double slowPower = -1;
    public static double offPower = 0;
    public static double reversePower = 1;
    public static double shortReverseTimeMs = 150;

    private final DcMotorEx intakeMotor;

    private final Telemetry telemetry;

    public Intake(Robot robot) {
        intakeMotor = robot.hardwareMap.get(DcMotorEx.class, "intake");
        // Telemetry is not required, but is an extremely powerful tool for debugging
        telemetry = robot.telemetry;
    }
    
    // Example of creating a command within the subsystem
    public Command on() {
        return instant(() -> mode = Mode.ON).requiring(intakeMotor);
    }

    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(intakeMotor);
    }

    public Command reverse() {
        return instant(() -> mode = Mode.REVERSE).requiring(intakeMotor);
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }

    public void slowDown() {
        slowMode = true;
    }

    public void speedUp() {
        slowMode = false;
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case ON:
                    // setPower(condition ? valueIfTrue : valueIfFalse)
                    intakeMotor.setPower(slowMode ? slowPower : fastPower);
                    break;
                case OFF:
                    intakeMotor.setPower(offPower);
                    break;
                case REVERSE:
                    intakeMotor.setPower(reversePower);
                    break;
            }

            telemetry.addData("Intake Current", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        });
    }

    enum Mode {
        ON,
        OFF,
        REVERSE
    }
}
```
{% endtab %}

{% tab title="Flywheel.java" %}
```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.math.WaveLength;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public class Flywheel {
    public static double kP = 0.01;
    public static double kS = 0.065;
    public static double kV = 0.000365;
    public static int velocityTolerance = 25;
    public static boolean override = false;
    public static double overrideTarget = 1000;
    private final DcMotorEx flywheelMotorTop;
    private final DcMotorEx flywheelMotorBottom;
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private boolean tempOverride = false;
    private boolean on = false;
    private double target = 0;

    public Flywheel(Robot robot) {
        flywheelMotorTop = robot.hardwareMap.get(DcMotorEx.class, "flywheelTop");
        flywheelMotorBottom = robot.hardwareMap.get(DcMotorEx.class, "flywheelBottom");
        flywheelMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = robot.telemetry;
        drivetrain = robot.drivetrain;
    }

    public void setTarget(double target) {
        this.target = target;
        tempOverride = true;
    }

    public void useInterpolation() {
        tempOverride = false;
    }

    private void setPower(double power) {
        flywheelMotorTop.setPower(power);
        flywheelMotorBottom.setPower(power);
    }

    private double getVelocity() {
        return flywheelMotorBottom.getVelocity();
    }

    public boolean atTarget() {
        return Math.abs(target - getVelocity()) <= velocityTolerance;
    }

    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
    }

    public void toggle() {
        on = !on;
        if (on) turnOn();
        else turnOff();
    }

    public Command periodic() {
        return infinite(() -> {
            if (on) {
                // Interpolation involves consideration of their robot's position
                // and orientation. For now, you can ignore this.
                target = override ? overrideTarget : (tempOverride ? target : WaveLength.getVelocityWithInterpolation(TurretLocation.getTurretPose(drivetrain.getPose()), Alliance.current));
                // This is a great way of doing PID without using a function!
                setPower(kP * (target - getVelocity()) + kV * target + kS * Math.signum(target));
            } else {
                target = 0;
                setPower(0);
            }

            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", target);
            telemetry.addData("Flywheel Power", flywheelMotorBottom.getPower());
        });
    }
}
```
{% endtab %}
{% endtabs %}
