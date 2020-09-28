package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import com.sun.accessibility.internal.resources.accessibility;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.function.Function;

public class DuncanRobot05Behavior implements Robot05Behavior
{
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private double x = 0.5, y = 0.5;
   private double heading = 0.0;
   private double velocity;
   private double wallDistance;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> closestFlag;

   public DuncanRobot05Behavior()
   {
   }

   @Override
   public void senseVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   @Override
   public void senseHeading(double heading)
   {
      this.heading = heading;
   }

   @Override
   public void senseWallRangeInBodyFrame(Vector2D vectorToWallInBodyFrame, double wallDistance) // to wall directly in front
   {
      this.wallDistance = wallDistance;
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void droppedFlag(int flagId)
   {
      if (carrying != flagId)
      {
         carrying = -1;
         LogTools.info("Dropped flag: {} Going for: {}", flagId, currentFlagId);
      }
   }

   @Override
   public void pickedUpFlag(int id)
   {
      carrying = id;
      LogTools.info("Picked up: {} Going for: {}", carrying, currentFlagId);
   }

   @Override
   public void deliveredFlag(int flagId)
   {
      if (carrying == flagId)
      {
         carrying = -1;
         LogTools.info("Goal! Flag: {}", currentFlagId);
         if (currentFlagId < 5)
         {
            currentFlagId++;
         }
         else
         {
            currentFlagId = 1;
         }
         LogTools.info("Next flag: {}", currentFlagId);
      }
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> vectorToInBodyFrameAndIdOfClosestFlag)
   {
      this.closestFlag = vectorToInBodyFrameAndIdOfClosestFlag;
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      this.mousePressedX = mousePressedX;
      this.mousePressedY = mousePressedY;
   }

   int currentFlagId = 1;
   int carrying = -1;

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      // 0 heading is y+ (up)
      double dt = 0.01;
      x += dt * velocity * -Math.sin(heading);
      y += dt * velocity * Math.cos(heading);

      double fieldGraduation = 1.5;
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Point2D me = new Point2D(x, y);
      Vector2D attractionVector = new Vector2D();

      Vector2D meToMouse = fieldVector(me, mouse, distance -> 10.0 * Math.pow(distance, 1.5));

      Point2D center = new Point2D(5.0, 5.0);
      Vector2D meToCenter = fieldVector(me, center, distance -> 2.0 * Math.pow(distance, 1.5));

      Line2D left = new Line2D(0.0, 0.0, 0.0, 1.0);
      Line2D right = new Line2D(10.0, 0.0, 0.0, 1.0);
      Line2D bottom = new Line2D(0.0, 0.0, 1.0, 0.0);
      Line2D top = new Line2D(0.0, 10.0, 1.0, 0.0);
      Vector2D boundaryRepulsion = new Vector2D();
      double boundaryStrength = 2.0;
      Point2D closestLeft = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, left.getPoint(), left.getDirection());
      Point2D closestRight = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, right.getPoint(), right.getDirection());
      Point2D closestBottom = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, bottom.getPoint(), bottom.getDirection());
      Point2D closestTop = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, top.getPoint(), top.getDirection());
      boundaryRepulsion.add(fieldVector(closestLeft, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));
      boundaryRepulsion.add(fieldVector(closestRight, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));
      boundaryRepulsion.add(fieldVector(closestBottom, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));
      boundaryRepulsion.add(fieldVector(closestTop, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));

      Vector2D predatorRepulsion = new Vector2D();
      for (Pair<Point2D, Vector2D> predator : locationOfAllPredators)
      {
         predatorRepulsion.add(fieldVector(predator.getLeft(), me, distance -> 6.0 / Math.pow(distance, fieldGraduation)));
      }
//      predatorRepulsion.scale(1.0 / locationOfAllPredators.size());

      Vector2D foodAttraction = new Vector2D();
      for (Pair<Point2D, Vector2D> food : locationOfAllFood)
      {
         foodAttraction.add(fieldVector(me, food.getLeft(), distance -> 0.5 / Math.pow(distance, 1.5)));
      }

      if (closestFlag != null)
      {
         Vector2D flagField = new Vector2D();
         if (closestFlag != null && closestFlag.getRight() == currentFlagId)
         {
            if (carrying != currentFlagId) // toward flag to pick up
            {
               flagField.add(fieldVector(me, closestFlag.getLeft(), distance -> 6.0 / Math.pow(distance, fieldGraduation)));
            }
         }
         else
         {
            flagField.add(fieldVector(closestFlag.getLeft(), me, distance -> 3.0 / Math.pow(distance, 2.0)));
         }

         if (carrying == currentFlagId) // set to goal
         {
            flagField.add(fieldVector(me, new Point2D(9.0, 9.0), distance -> 15.0 / Math.pow(distance, 0.5)));
         }
         attractionVector.add(flagField);
      }

//      attractionVector.add(meToMouse);
//      attractionVector.add(meToCenter);
      attractionVector.add(boundaryRepulsion);
//      attractionVector.add(predatorRepulsion);
//      attractionVector.add(foodAttraction);

      double desiredSpeed = attractionVector.length();

      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                    headingVector.getY(),
                                                                                    attractionVector.getX(),
                                                                                    attractionVector.getY());

      double acceleration = (1.0 * (desiredSpeed - velocity));

      double angularVelocity = (velocity - lastVelocity) / dt;
      double turnRate = (5.0 * angleToAttraction) + (-0.5 * angularVelocity);
      lastVelocity = velocity;

      LogTools.info("me: {} accel: {} turn: {}", me, acceleration, turnRate);

      if (Double.isNaN(acceleration)) acceleration = 0.0;
      if (Double.isNaN(turnRate)) turnRate = 0.0;

      if (me.getX() < 0.001 || me.getY() < 0.001)
      {
         return new double[] {0.1, 0.1};
      }

      return new double[] {acceleration, turnRate};
   }

   double lastVelocity = 0.0;

   private Vector2D fieldVector(Tuple2DReadOnly from, Tuple2DReadOnly to, Function<Double, Double> magnitude)
   {
      Vector2D vector = new Vector2D(to);
      vector.sub(from);
      double distance = vector.length();
      vector.normalize();
      vector.scale(magnitude.apply(distance));
      return vector;
   }

   @Override
   public boolean getDropFlag()
   {
      return ((x > 8.0) && (y > 8.0));
   }
}
