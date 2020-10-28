package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

public class RobotBehaviorActuators
{
   private double turnRate;
   private double acceleration;
   private boolean dropFlag;
   
   public double getTurnRate()
   {
      return turnRate;
   }
   public void setTurnRate(double turnRate)
   {
      this.turnRate = turnRate;
   }
   public double getAcceleration()
   {
      return acceleration;
   }
   public void setAcceleration(double acceleration)
   {
      this.acceleration = acceleration;
   }
   
   public boolean getDropFlag()
   {
      return dropFlag;
   }
   
   public void setDropFlag(boolean dropFlag)
   {
      this.dropFlag = dropFlag;
   }
}
