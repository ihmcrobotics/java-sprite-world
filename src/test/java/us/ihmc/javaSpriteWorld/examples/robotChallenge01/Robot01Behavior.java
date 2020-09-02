package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

public interface Robot01Behavior
{
   public abstract void senseFood(double x, double y);
   public abstract void senseGlobalLocation(double x, double y);
   public abstract void senseVelocity(double velocity);
   public abstract void senseHeading(double heading);
   public abstract void senseMousePressed(double mousePressedX, double mousePressedY);
   
   public abstract double[] getAccelerationAndTurnRate();

}
