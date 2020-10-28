package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotBehaviorSensors
{
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood = new ArrayList<Triple<Integer,Point2D,Vector2D>>();

   public ArrayList<Triple<Integer, Point2D, Vector2D>> getLocationOfAllFood()
   {
      return locationOfAllFood;
   }

   public void setLocationOfAllFood(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
      this.locationOfAllFood.clear();
      this.locationOfAllFood.addAll(locationOfAllFood);
   }

   
   
}
