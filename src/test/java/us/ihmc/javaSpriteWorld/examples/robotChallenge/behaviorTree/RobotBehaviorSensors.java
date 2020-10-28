package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class RobotBehaviorSensors
{
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFoodInBodyFrame = new ArrayList<Triple<Integer,Point2D,Vector2D>>();
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators = new ArrayList<>();

   private double heading;
   private double velocity;
   private double globalX, globalY;
   
   private double score;
   private double health;
   private double elapsedTime;
   
   public ArrayList<Triple<Integer, Point2D, Vector2D>> getLocationOfAllFoodInBodyFrame()
   {
      return locationOfAllFoodInBodyFrame;
   }

   public void setLocationOfAllFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)

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
 
}
