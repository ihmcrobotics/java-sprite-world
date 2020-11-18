package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import static us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree.RobotBehaviorTools.bodyToWorld;

public class RobotBehaviorSensors
{
   private final ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame = new ArrayList<Triple<Integer,Point2D,Vector2D>>();
   private final ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators = new ArrayList<>();
   private final ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame = new ArrayList<Pair<Vector2D,Double>>();
   private ImmutablePair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag;

   private double heading;
   private double velocity;
   private double globalX = 1.5, globalY = 1.5;
   
   private double score;
   private double health;
   private double elapsedTime;

   private AtomicInteger sensedDroppedFlag = new AtomicInteger(-1);
   private AtomicInteger sensedPickedUpFlag = new AtomicInteger(-1);
   private AtomicInteger sensedDeliveredFlag = new AtomicInteger(-1);
   private AtomicInteger sensedCarryingFlag = new AtomicInteger(-1);

   private boolean simulationReset = false;

   double closestPredatorDistance;
   double closestFoodDistance;

   public double getClosestPredatorDistance()
   {
      return closestPredatorDistance;
   }

   public double getClosestFoodDistance()
   {
      return closestFoodDistance;
   }

   public void processSensorData()
   {
      closestPredatorDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < getLocationOfAllPredators().size(); i++)
      {
         Point2D predatorInBodyFrame = getLocationOfAllPredators().get(i).getLeft();
         double distance = EuclidCoreTools.norm(predatorInBodyFrame.getX(), predatorInBodyFrame.getY());

         if (distance < closestPredatorDistance)
         {
            closestPredatorDistance = distance;
         }
      }

      closestFoodDistance = Double.POSITIVE_INFINITY;
      for (Triple<Integer, Point2D, Vector2D> food : getLocationOfAllFoodInBodyFrame())
      {
         Point2D foodLocation = bodyToWorld(this, food.getMiddle());
         double distance = getGlobalPosition().distance(foodLocation);
         if (distance < closestFoodDistance)
         {
            closestFoodDistance = distance;
         }
      }
   }

   public void setSimulationReset()
   {
      simulationReset = true;
   }

   public boolean isSimulationReset()
   {
      return simulationReset;
   }

   public void clearSimulationReset()
   {
      simulationReset = false;
   }

   public ArrayList<Triple<Integer, Point2D, Vector2D>> getLocationOfAllFoodInBodyFrame()
   {
      return locationOfAllFoodInBodyFrame;
   }

   public ArrayList<Pair<Point2D, Vector2D>> getLocationOfAllPredators()
   {
      return locationOfAllPredators;
   }

   public void setLocationOfAllFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFoodInBodyFrame.clear();
      this.locationOfAllFoodInBodyFrame.addAll(locationOfAllFood);
   }

   public void setLocationOfAllPredators(ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators)
   {
      this.locationOfAllPredators.clear();
      this.locationOfAllPredators.addAll(locationOfAllPredators);
   }

   public void setHeading(double heading)
   {
      this.heading = heading;
      
   }
   
   public double getHeading()
   {
      return heading;
   }

   public void setGlobalPosition(double x, double y)
   {
      this.globalX = x;
      this.globalY = y;
   }

   public Point2DReadOnly getGlobalPosition()
   {
      return new Point2D(globalX, globalY);
   }

   public double getVelocity()
   {
      return velocity;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   public double getScore()
   {
      return score;
   }

   public void setScore(double score)
   {
      this.score = score;
   }

   public double getHealth()
   {
      return health;
   }

   public void setHealth(double health)
   {
      this.health = health;
   }

   public double getElapsedTime()
   {
      return elapsedTime;
   }

   public void setElapsedTime(double elapsedTime)
   {
      this.elapsedTime = elapsedTime;
   }

   public ArrayList<Pair<Vector2D, Double>> getVectorsAndDistancesToWallInBodyFrame()
   {
      return vectorsAndDistancesToWallInBodyFrame;
   }

   public void setVectorsAndDistancesToWallInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      this.vectorsAndDistancesToWallInBodyFrame.clear();
      this.vectorsAndDistancesToWallInBodyFrame.addAll(vectorsAndDistancesToWallInBodyFrame);
   }

   public Pair<Point2D, Integer> getPositionInBodyFrameAndIdOfClosestFlag()
   {
      return positionInBodyFrameAndIdOfClosestFlag;
   }

   public void setPositionInBodyFrameAndIdOfClosestFlag(Pair<Point2D, Integer> positionInBodyFrameAndIdOfClosestFlag)
   {
      this.positionInBodyFrameAndIdOfClosestFlag = new ImmutablePair<Point2D, Integer>(positionInBodyFrameAndIdOfClosestFlag.getLeft(), positionInBodyFrameAndIdOfClosestFlag.getRight());
   }

   public void senseDroppedFlag(int flagId)
   {
      sensedDroppedFlag.set(flagId);
   }

   public void sensePickedUpFlag(int id)
   {
      sensedPickedUpFlag.set(id);
   }

   public void senseDeliveredFlag(int flagId)
   {
      sensedDeliveredFlag.set(flagId);
   }

   public void senseCarryingFlag(int flagId)
   {
      sensedCarryingFlag.set(flagId);
   }

   public int pollSensedDroppedFlag()
   {
      return sensedDroppedFlag.getAndSet(-1);
   }

   public int pollSensedPickedUpFlag()
   {
      return sensedPickedUpFlag.getAndSet(-1);
   }

   public int pollSensedDeliveredFlag()
   {
      return sensedDeliveredFlag.getAndSet(-1);
   }

   public int pollSensedCarryingFlag()
   {
      return sensedCarryingFlag.getAndSet(-1);
   }
}
