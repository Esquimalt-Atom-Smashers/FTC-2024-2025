
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;


import java.lang.Thread;
public class ArmExtensionSubSys extends SubsystemBase {
    // Declare the motor
    private DcMotor armMotor;
    private HardwareMap hardwareMap;

    public void ExtendArm() {

        // Initialize hardware
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");  // Replace "motor" with your motor's name

    }
}
