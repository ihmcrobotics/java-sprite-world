package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class FlagList
{
   private final ArrayList<Flag> capturedFlags = new ArrayList<Flag>();
   private final ArrayList<Flag> flagsInPlay = new ArrayList<Flag>();
   private final ArrayList<Flag> deliveredFlags = new ArrayList<Flag>();

   private final HashMap<Sprite, Flag> map = new HashMap<Sprite, Flag>();
   
   public FlagList()
   {
   }

   public void createAFlag(int id, Random random, double xMax, double yMax, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      double x = randomDoubleBetween(random, xMax * 0.1, xMax * 0.9);
      double y = randomDoubleBetween(random, yMax * 0.3, yMax * 0.9);

      Flag flag = new Flag(id);
      flag.setLocation(x, y);
      addFlag(flag, spriteWorld, collisionGroup);
   }

   private double randomDoubleBetween(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   public void addFlag(Flag flag, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      spriteWorld.addSprite(flag.getSprite());
      collisionGroup.addSprite(flag.getSprite());

      if (!flagsInPlay.contains(flag))
      {
         flagsInPlay.add(flag);
      }

      flag.getSprite().show();

      if (map.get(flag.getSprite()) == null)
      {
         map.put(flag.getSprite(), flag);
      }
   }

   public void reset(Random random, double xMax, double yMax, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      ArrayList<Flag> flagsToAdd = new ArrayList<Flag>();
      
      flagsToAdd.addAll(deliveredFlags);
      flagsToAdd.addAll(capturedFlags);
      flagsToAdd.addAll(flagsInPlay);
      
      deliveredFlags.clear();
      capturedFlags.clear();
      flagsInPlay.clear();

      for (Flag flag : flagsToAdd)
      {
         double x = randomDoubleBetween(random, xMax * 0.1, xMax * 0.9);
         double y = randomDoubleBetween(random, yMax * 0.3, yMax * 0.9);

         flag.setLocation(x, y); 
         addFlag(flag, spriteWorld, collisionGroup);
 
//         System.out.println("Resetting flag " + flag.getId() + " to " + x + ", " + y);
      }
   }
   
   public void capturedFlag(Flag flag, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      collisionGroup.removeSprite(flag.getSprite());
      spriteWorld.removeSprite(flag.getSprite());
      flagsInPlay.remove(flag);
      flag.getSprite().hide();

      map.remove(flag.getSprite(), flag);
      capturedFlags.add(flag);
   }
   
   public void deliveredFlag(Flag flag)
   {
      capturedFlags.remove(flag);
      deliveredFlags.add(flag);

      map.remove(flag.getSprite(), flag);
   }
   
   public void droppedFlag(Flag droppedFlag, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      capturedFlags.remove(droppedFlag);
      addFlag(droppedFlag, spriteWorld, collisionGroup);
   }

   public Flag findFlag(Sprite sprite)
   {
      Flag flag = map.get(sprite);
      return flag;
   }

   public void doDynamicsAndUpdateSprites(double dt)
   {
      for (Flag flag : flagsInPlay)
      {
         flag.doDynamicsAndUpdateSprite(dt);
      }
   }

   public ArrayList<Pair<Point2D, Integer>> getLocationAndIdsOfAllFlags()
   {
      ArrayList<Pair<Point2D, Integer>> locationsAndIds = new ArrayList<Pair<Point2D, Integer>>();

      for (Flag flag : flagsInPlay)
      {
         Point2D location = new Point2D(flag.getX(), flag.getY());

         int id = flag.getId();

         locationsAndIds.add(new ImmutablePair<Point2D, Integer>(location, id));
      }

      return locationsAndIds;
   }
   
   public Pair<Point2D, Integer> getLocationAndIdOfClosestFlag(Point2D point)
   {
      ArrayList<Pair<Point2D, Integer>> locationAndIdsOfAllFlags = getLocationAndIdsOfAllFlags();
     
      double closestDistance = Double.POSITIVE_INFINITY;
      Pair<Point2D, Integer> closestFlag = null;
      
      for (Pair<Point2D, Integer> flag : locationAndIdsOfAllFlags)
      {
         Point2D location = flag.getLeft();
         double distance = location.distance(point);
         
         if (distance < closestDistance)
         {
            closestDistance = distance;
            closestFlag = flag;
         }
      }
      
      return closestFlag;
   }

}
