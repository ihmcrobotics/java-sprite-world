package us.ihmc.javaSpriteWorld.examples.robotChallenge06;

public class RobotChallenge06NoiseParameters
{
   private double robotVelocityNoiseMagnitude;
   private double robotVelocityBiasMagnitude;
   private double robotHeadingNoiseMagnitude;
   private double predatorPositionNoiseMagnitude;
   private double predatorVelocityNoiseMagnitude;
   private double foodPositionNoiseMagnitude;
   private double foodVelocityNoiseMagnitude;
   private double flagPositionNoiseMagnitude;
   private double wallDistanceNoiseMagnitude;

   public RobotChallenge06NoiseParameters()
   {
      setRobotVelocityNoiseMagnitude(1.0);
      setRobotVelocityBiasMagnitude(1.0);
      setRobotHeadingNoiseMagnitude(0.1);
      setPredatorPositionNoiseMagnitude(0.5);
      setPredatorVelocityNoiseMagnitude(1.0);
      setFoodPositionNoiseMagnitude(0.5);
      setFoodVelocityNoiseMagnitude(1.0);
      setFlagPositionNoiseMagnitude(0.5);
      setWallDistanceNoiseMagnitude(0.5);
   }

   public void removeAllNoise()
   {
      setRobotVelocityNoiseMagnitude(0.0);
      setRobotVelocityBiasMagnitude(0.0);
      setRobotHeadingNoiseMagnitude(0.0);
      setPredatorPositionNoiseMagnitude(0.0);
      setPredatorVelocityNoiseMagnitude(0.0);
      setFoodPositionNoiseMagnitude(0.0);
      setFoodVelocityNoiseMagnitude(0.0);
      setFlagPositionNoiseMagnitude(0.0);
      setWallDistanceNoiseMagnitude(0.0);
   }

   public double getRobotVelocityNoiseMagnitude()
   {
      return robotVelocityNoiseMagnitude;
   }

   public void setRobotVelocityNoiseMagnitude(double robotVelocityNoiseMagnitude)
   {
      this.robotVelocityNoiseMagnitude = robotVelocityNoiseMagnitude;
   }

   public double getRobotVelocityBiasMagnitude()
   {
      return robotVelocityBiasMagnitude;
   }

   public void setRobotVelocityBiasMagnitude(double robotVelocityBiasMagnitude)
   {
      this.robotVelocityBiasMagnitude = robotVelocityBiasMagnitude;
   }

   public double getRobotHeadingNoiseMagnitude()
   {
      return robotHeadingNoiseMagnitude;
   }

   public void setRobotHeadingNoiseMagnitude(double robotHeadingNoiseMagnitude)
   {
      this.robotHeadingNoiseMagnitude = robotHeadingNoiseMagnitude;
   }

   public double getPredatorPositionNoiseMagnitude()
   {
      return predatorPositionNoiseMagnitude;
   }

   public void setPredatorPositionNoiseMagnitude(double predatorPositionNoiseMagnitude)
   {
      this.predatorPositionNoiseMagnitude = predatorPositionNoiseMagnitude;
   }

   public double getPredatorVelocityNoiseMagnitude()
   {
      return predatorVelocityNoiseMagnitude;
   }

   public void setPredatorVelocityNoiseMagnitude(double predatorVelocityNoiseMagnitude)
   {
      this.predatorVelocityNoiseMagnitude = predatorVelocityNoiseMagnitude;
   }

   public double getFoodPositionNoiseMagnitude()
   {
      return foodPositionNoiseMagnitude;
   }

   public void setFoodPositionNoiseMagnitude(double foodPositionNoiseMagnitude)
   {
      this.foodPositionNoiseMagnitude = foodPositionNoiseMagnitude;
   }

   public double getFoodVelocityNoiseMagnitude()
   {
      return foodVelocityNoiseMagnitude;
   }

   public void setFoodVelocityNoiseMagnitude(double foodVelocityNoiseMagnitude)
   {
      this.foodVelocityNoiseMagnitude = foodVelocityNoiseMagnitude;
   }

   public double getFlagPositionNoiseMagnitude()
   {
      return flagPositionNoiseMagnitude;
   }

   public void setFlagPositionNoiseMagnitude(double flagPositionNoiseMagnitude)
   {
      this.flagPositionNoiseMagnitude = flagPositionNoiseMagnitude;
   }

   public double getWallDistanceNoiseMagnitude()
   {
      return wallDistanceNoiseMagnitude;
   }

   public void setWallDistanceNoiseMagnitude(double wallDistanceNoiseMagnitude)
   {
      this.wallDistanceNoiseMagnitude = wallDistanceNoiseMagnitude;
   }

}
