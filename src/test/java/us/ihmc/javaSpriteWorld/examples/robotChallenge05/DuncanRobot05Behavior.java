package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.*;
import java.util.function.Function;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class DuncanRobot05Behavior implements Robot05Behavior, Robot06Behavior
{
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private double x = 0.5, y = 0.5;
   private double heading = 0.0;
   private double velocity;
   private double wallDistance;
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> closestFlag;
   private Point2D me;

   private double lastVelocityForFilter = 0.0;
   private double lastHeadingForFilter = 0.0;
   private final ArrayDeque<Double> velocities = new ArrayDeque<>();
   private final ArrayDeque<Double> headings = new ArrayDeque<>();

   public DuncanRobot05Behavior()
   {
   }

   @Override
   public void senseVelocity(double velocity)
   {
      this.velocity = alphaFilter(velocity, lastVelocityForFilter, 0.1);
      LogTools.info("Alpha'd {} -> {}", velocity, this.velocity);
   }

   @Override
   public void senseHeading(double heading)
   {
      this.heading = alphaFilter(heading, lastHeadingForFilter, 0.1);
   }

   private double alphaFilter(double current, double last, double alpha)
   {
      return current + alpha * (current - last);
   }

   private double filter(double currentData, ArrayDeque<Double> dataHistory, double alpha)
   {
      // TODO: Low pass filter or alpha filter
      dataHistory.addFirst(currentData);
      while (dataHistory.size() > 15)
         dataHistory.removeLast();

      double average = 0.0;
      for (Double dataPoint : dataHistory)
      {
         average += dataPoint;
      }
      average /= dataHistory.size();
      return average;
   }

   @Override
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      // 0, 1 is straight ahead
      this.wallDistance = wallDistance;
   }

   @Override
   public void senseHitWall()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      if (carrying != flagId)
      {
         carrying = -1;
         LogTools.info("Dropped flag: {} Going for: {}", flagId, currentFlagId);
      }
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      carrying = id;
      LogTools.info("Picked up: {} Going for: {}", carrying, currentFlagId);
   }

   @Override
   public void senseDeliveredFlag(int flagId)
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
      Vector2D headingVector = new Vector2D(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(heading);
      transform.transform(headingVector);

      // 0 heading is y+ (up)
      double dt = 0.01;
      x += dt * velocity * -Math.sin(heading);
      y += dt * velocity * Math.cos(heading);

      double fieldGraduation = 1.5;
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      me = new Point2D(x, y);
      Vector2D attractionVector = new Vector2D();

      Vector2D meToMouse = fieldVector(me, mouse, distance -> 10.0 * Math.pow(distance, 1.5));
      Point2D center = new Point2D(5.0, 5.0);
      Vector2D meToCenter = fieldVector(me, center, distance -> 2.0 * Math.pow(distance, 1.5));

      Line2D left = new Line2D(0.0, 0.0, 0.0, 1.0);
      Line2D right = new Line2D(10.0, 0.0, 0.0, 1.0);
      Line2D bottom = new Line2D(0.0, 0.0, 1.0, 0.0);
      Line2D top = new Line2D(0.0, 10.0, 1.0, 0.0);
      List<Line2D> walls = Arrays.asList(left, right, bottom, top);

      Vector2D toWallAhead = new Vector2D(100.0, 100.0);
      for (Line2D wall : walls)
      {
         Point2D intersection = new Point2D();
         wall.intersectionWith(new Line2D(me, headingVector), intersection);
         Vector2D toWall = new Vector2D();
         toWall.sub(intersection, me);
         if (toWall.dot(headingVector) > 0.0 && toWall.length() < toWallAhead.length())
         {
            toWallAhead = toWall;
         }
      }
      double wallDistanceError = wallDistance - toWallAhead.length();
//      LogTools.info("Wall distance error: {}", wallDistanceError);

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
         predatorRepulsion.add(fieldVector(bodyToWorld(predator.getLeft()), me, distance -> 6.0 / Math.pow(distance, fieldGraduation)));
      }
//      predatorRepulsion.scale(1.0 / locationOfAllPredators.size());

      Vector2D foodAttraction = new Vector2D();
      for (Triple<Integer, Point2D, Vector2D> food : locationOfAllFood)
      {
         foodAttraction.add(fieldVector(me, bodyToWorld(food.getMiddle()), distance -> 0.5 / Math.pow(distance, 1.5)));
      }

      if (closestFlag != null)
      {
         Vector2D flagField = new Vector2D();
         if (closestFlag != null && closestFlag.getRight() == currentFlagId)
         {
            if (carrying != currentFlagId) // toward flag to pick up
            {
               flagField.add(fieldVector(me, bodyToWorld(closestFlag.getLeft()), distance -> 6.0 / Math.pow(distance, fieldGraduation)));
            }
         }
         else
         {
            flagField.add(fieldVector(bodyToWorld(closestFlag.getLeft()), me, distance -> 3.0 / Math.pow(distance, 2.0)));
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
      attractionVector.add(predatorRepulsion);
      attractionVector.add(foodAttraction);

      double desiredSpeed = attractionVector.length();

      double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                    headingVector.getY(),
                                                                                    attractionVector.getX(),
                                                                                    attractionVector.getY());

      double acceleration = (1.0 * (desiredSpeed - velocity));

      double angularVelocity = (velocity - lastVelocity) / dt;
      double turnRate = (5.0 * angleToAttraction) + (-0.5 * angularVelocity);
      lastVelocity = velocity;

      if (Double.isNaN(acceleration)) acceleration = 0.0;
      if (Double.isNaN(turnRate)) turnRate = 0.0;

      return new double[] {acceleration, turnRate};
   }

   double lastVelocity = 0.0;

   private Point2D bodyToWorld(Tuple2DReadOnly pointInBody)
   {
      Point2D pointInWorld = new Point2D(pointInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-heading, 0.0, 0.0);
      pointInWorld.applyInverseTransform(transform);
      pointInWorld.add(me);
      return pointInWorld;
   }

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

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {      
   }
}
