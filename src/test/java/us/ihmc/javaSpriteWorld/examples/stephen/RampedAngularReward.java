package us.ihmc.javaSpriteWorld.examples.stephen;

import us.ihmc.euclid.tools.EuclidCoreTools;

public class RampedAngularReward implements ObjectResponseDescription
{
   // Higher weight rewards/penalties for objects in front of the robot
   private static final double rewardScaleWhenBehindRobot = 0.7;

   private final double headingOfObjectInBodyFrame;
   private final double angularRange;
   private final double rewardWhenFacingObject;

   public RampedAngularReward(double headingOfObjectInBodyFrame, double angularRange, double rewardWhenFacingObject)
   {
      this.headingOfObjectInBodyFrame = headingOfObjectInBodyFrame;
      this.angularRange = angularRange;
      this.rewardWhenFacingObject = rewardWhenFacingObject;
   }

   @Override
   public double getRewardAtAngle(double headingInBodyFrame)
   {
      double angularDifference = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(headingInBodyFrame, headingOfObjectInBodyFrame));
      if (angularDifference > angularRange)
      {
         return 0.0;
      }
      else
      {
         double multiplier = EuclidCoreTools.interpolate(1.0, rewardScaleWhenBehindRobot, Math.abs(headingOfObjectInBodyFrame) / Math.PI);
         return multiplier * rewardWhenFacingObject * (1.0 - angularDifference / angularRange);
      }
   }
}
