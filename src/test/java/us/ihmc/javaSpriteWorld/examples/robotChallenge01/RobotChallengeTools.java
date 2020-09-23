package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotChallengeTools
{
   public static Point2D computePositionInRobotBodyFrame(Point2D robotLocationInWorld, Point2D pointOfInterestInWorld, double robotHeading)
   {
      Vector2D vectorFromRobotInWorld = computeVectorFromRobotInWorld(robotLocationInWorld, pointOfInterestInWorld);
      Point2D vectorInBody = computePositionInBodyFrame(vectorFromRobotInWorld, robotHeading);
      return vectorInBody;
   }

   public static Vector2D computeVectorFromRobotInWorld(Point2D robotLocationInWorld, Point2D pointOfInterestInWorld)
   {
      Vector2D vectorFromRobotInWorld = new Vector2D(pointOfInterestInWorld);
      vectorFromRobotInWorld.sub(robotLocationInWorld);

      return vectorFromRobotInWorld;
   }

   public static Point2D computePositionInBodyFrame(Vector2D vectorInWorld, double robotHeading)
   {
      Point2D vectorInBody = new Point2D(vectorInWorld);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(robotHeading, 0.0, 0.0);
      vectorInBody.applyInverseTransform(transform);

      return vectorInBody;
   }
   
   public static Vector2D computeVectorInBodyFrame(Vector2D vectorInWorld, double robotHeading)
   {
      Vector2D vectorInBody = new Vector2D(vectorInWorld);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(robotHeading, 0.0, 0.0);
      vectorInBody.applyInverseTransform(transform);

      return vectorInBody;
   }

   public static ArrayList<Pair<Point2D, Vector2D>> convertFromWorldToBodyFrame(Point2D robotLocationInWorld, ArrayList<Pair<Point2D, Vector2D>> locationAndVelocityInWorldFrame, double robotHeading)
   {
      ArrayList<Pair<Point2D, Vector2D>> positionAndVelocityInBodyFrame = new ArrayList<Pair<Point2D,Vector2D>>();
      
      for (Pair<Point2D, Vector2D> locationAndVelocity : locationAndVelocityInWorldFrame)
      {
         Point2D locationInWorld = locationAndVelocity.getLeft();
         Vector2D velocityInWorld = locationAndVelocity.getRight();
         
         Point2D locationInBody = computePositionInRobotBodyFrame(robotLocationInWorld, locationInWorld, robotHeading);
         Vector2D velocityInBody = computeVectorInBodyFrame(velocityInWorld, robotHeading);

         Pair<Point2D, Vector2D> locationAndVelocityInBody = new ImmutablePair<Point2D, Vector2D>(locationInBody, velocityInBody);
         positionAndVelocityInBodyFrame.add(locationAndVelocityInBody);
      }
      
      return positionAndVelocityInBodyFrame;
   }
}
