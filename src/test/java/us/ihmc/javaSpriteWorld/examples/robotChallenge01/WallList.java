package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteWorld;

public class WallList
{
   private final ArrayList<Wall> walls = new ArrayList<Wall>();
   private final HashMap<Sprite, Wall> map = new HashMap<Sprite, Wall>();

   public WallList()
   {
   }

   public void createWall(Point2D pointOne, Point2D pointTwo, SpriteWorld spriteWorld, SpriteCollisionGroup collisionGroup)
   {
      Wall wall = new Wall(pointOne, pointTwo);
      spriteWorld.addSprite(wall.getSprite());
      collisionGroup.addSprite(wall.getSprite());

      map.put(wall.getSprite(), wall);
      walls.add(wall);
   }

   public Wall findWall(Sprite sprite)
   {
      Wall wall = map.get(sprite);
      return wall;
   }

   public ArrayList<LineSegment2DReadOnly> getWallLineSegments()
   {
      ArrayList<LineSegment2DReadOnly> wallLineSegments = new ArrayList<LineSegment2DReadOnly>();
      
      for (Wall wall : walls)
      {
         LineSegment2DReadOnly lineSegment = wall.getLineSegment();
         wallLineSegments.add(lineSegment);
      }
      
      return wallLineSegments;
   }

   //   public ArrayList<Pair<Point2D, Vector2D>> getLocationAndVelocityOfAllPredators()
   //   {
   //      ArrayList<Pair<Point2D, Vector2D>> locationsAndVelocities = new ArrayList<Pair<Point2D, Vector2D>>();
   //
   //      for (Predator01 predator : predatorsInPlay)
   //      {
   //         Point2D location = new Point2D(predator.getX(), predator.getY());
   //
   //         double heading = predator.getHeading();
   //         double speed = predator.getSpeed();
   //
   //         Vector2D velocity = new Vector2D(speed * Math.cos(heading), speed * Math.sin(heading));
   //         locationsAndVelocities.add(new ImmutablePair<Point2D, Vector2D>(location, velocity));
   //      }
   //
   //      return locationsAndVelocities;
   //   }

}
