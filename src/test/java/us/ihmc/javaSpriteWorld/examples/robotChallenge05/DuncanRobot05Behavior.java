package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.function.Function;

public class DuncanRobot05Behavior implements Robot05Behavior
{
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private double x = 0.0, y = 0.0;
   private double heading = 0.0;
   private double velocity;
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> closestFlag;

   public DuncanRobot05Behavior()
   {
   }

   @Override
   public void senseGlobalLocation(double x, double y)
   {
      this.x = x;
      this.y = y;
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
   public void senseFood(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredators(ArrayList<Pair<Vector2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators = locationOfAllPredators;
   }

   @Override
   public void droppedFlag(int flagId)
   {
      if (carrying != flagId)
         carrying = -1;
   }

   @Override
   public void pickedUpFlag(int id)
   {
      carrying = id;
   }

   @Override
   public void deliveredFlag(int flagId)
   {
      if (carrying == flagId)
         carrying = -1;
   }

   @Override
   public void senseClosestFlag(Pair<Point2D, Integer> locationAndIdsOfClosestFlag)
   {
      this.closestFlag = locationAndIdsOfClosestFlag;
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
      double fieldGraduation = 1.5;
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Point2D me = new Point2D(x, y);

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
      for (Pair<Vector2D, Vector2D> predator : locationOfAllPredators)
      {
         predatorRepulsion.add(fieldVector(predator.getLeft(), me, distance -> 6.0 / Math.pow(distance, fieldGraduation)));
      }
//      predatorRepulsion.scale(1.0 / locationOfAllPredators.size());

      Vector2D foodAttraction = new Vector2D();
      for (Pair<Vector2D, Vector2D> food : locationOfAllFood)
      {
         foodAttraction.add(fieldVector(me, food.getLeft(), distance -> 0.5 / Math.pow(distance, 1.5)));
      }

      Vector2D flagField = new Vector2D();
      if (closestFlag.getRight() == currentFlagId)
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

      Vector2D attractionVector = new Vector2D();
//      attractionVector.add(meToMouse);
//      attractionVector.add(meToCenter);
      attractionVector.add(boundaryRepulsion);
      attractionVector.add(predatorRepulsion);
      attractionVector.add(foodAttraction);
      attractionVector.add(flagField);

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

      double angularVelocity = (velocity - lastVelocity) / 0.01;
      double turnRate = (5.0 * angleToAttraction) + (-0.5 * angularVelocity);
      lastVelocity = velocity;

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

   @Override
   public void senseWallRange(Vector2D vectorToWallInBodyFrame, double wallDistance)
   {      
   }
}
