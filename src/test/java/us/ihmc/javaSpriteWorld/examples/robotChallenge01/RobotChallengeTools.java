package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotChallengeTools
{
   public static Vector2D computeVectorFromRobotInBody(Point2D robotLocationInWorld, Point2D pointOfInterestInWorld, double robotHeading)
   {
      Vector2D vectorFromRobotInWorld = computeVectorFromRobotInWorld(robotLocationInWorld, pointOfInterestInWorld);
      Vector2D vectorInBody = computeVectorInBody(vectorFromRobotInWorld, robotHeading);
      return vectorInBody;
   }

   public static Vector2D computeVectorFromRobotInWorld(Point2D robotLocationInWorld, Point2D pointOfInterestInWorld)
   {
      Vector2D vectorFromRobotInWorld = new Vector2D(pointOfInterestInWorld);
      vectorFromRobotInWorld.sub(robotLocationInWorld);

      return vectorFromRobotInWorld;
   }

   public static Vector2D computeVectorInBody(Vector2D vectorInWorld, double robotHeading)
   {
      Vector2D vectorInBody = new Vector2D(vectorInWorld);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(robotHeading, 0.0, 0.0);
      vectorInBody.applyInverseTransform(transform);

      return vectorInBody;
   }
}
