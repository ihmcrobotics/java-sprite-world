package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class RobotBehaviorSensors
{
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = new ArrayList<Triple<Integer,Point2D,Vector2D>>();
   private double heading;
   private double globalX, globalY;
   
   public ArrayList<Triple<Integer, Point2D, Vector2D>> getLocationOfAllFood()
   {
      return locationOfAllFood;
   }

   public void setLocationOfAllFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood.clear();
      this.locationOfAllFood.addAll(locationOfAllFood);
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

   
   
}
