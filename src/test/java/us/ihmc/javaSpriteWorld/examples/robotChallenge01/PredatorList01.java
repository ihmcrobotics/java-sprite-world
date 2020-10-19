package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class PredatorList01
{
   private final ArrayList<Predator01> predatorsInPlay = new ArrayList<Predator01>();
   private final ArrayList<Predator01> recycledPredators = new ArrayList<Predator01>();
   private final HashMap<Sprite, Predator01> map = new HashMap<Sprite, Predator01>();

   public PredatorList01()
   {
   }

   public void createAPredator(Random random, double xMax, double yMax, double maximumSpeed, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      Predator01 predator;

      if (!recycledPredators.isEmpty())
      {
         predator = recycledPredators.remove(0);
         predator.teleportToRandomLocation(random);
      }
      else
      {
         predator = new Predator01(random, xMax, yMax, maximumSpeed);
         spriteWorld.addSprite(predator.getSprite());
         collisionGroup.addSprite(predator.getSprite());
      }

      predatorsInPlay.add(predator);

      predator.getSprite().show();
      map.put(predator.getSprite(), predator);
   }

   public void reset(Random random)
   {
      for (Predator01 predator : predatorsInPlay)
      {
         predator.teleportToRandomLocation(random);
      }

      for (Predator01 predator : recycledPredators)
      {
         predator.teleportToRandomLocation(random);
      } 
   }
   
   public void removePredator(Predator01 predator)
   {
      predatorsInPlay.remove(predator);
      predator.getSprite().hide();
      if (!recycledPredators.contains(predator))
      {
         recycledPredators.add(predator);
      }

      map.remove(predator.getSprite(), predator);
   }

   public Predator01 findPredator(Sprite sprite)
   {
      Predator01 predator = map.get(sprite);
      return predator;
   }

   public void doDynamicsAndUpdateSprites(Point2D robotPosition, Vector2D robotVelocity, double dt)
   {
      for (Predator01 predator : predatorsInPlay)
      {
         predator.doDynamicsAndUpdateSprite(robotPosition, robotVelocity, dt);
      }
   }

   public ArrayList<Pair<Point2D, Vector2D>> getLocationAndVelocityOfAllPredators()
   {
      ArrayList<Pair<Point2D, Vector2D>> locationsAndVelocities = new ArrayList<Pair<Point2D, Vector2D>>();

      for (Predator01 predator : predatorsInPlay)
      {
         Point2D location = new Point2D(predator.getX(), predator.getY());

         double heading = predator.getHeading();
         double speed = predator.getSpeed();

         Vector2D velocity = new Vector2D(speed * Math.cos(heading), speed * Math.sin(heading));
         locationsAndVelocities.add(new ImmutablePair<Point2D, Vector2D>(location, velocity));
      }

      return locationsAndVelocities;
   }

}

