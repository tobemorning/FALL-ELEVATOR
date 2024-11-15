package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.RGBConstants;

public class RGBSubsystem {

 private Spark controller = new Spark(0);

 public RGBSubsystem() {

 }

 public void setColor() {
  controller.set(-0.29);
  System.out.println("RGBING");
 }
}
