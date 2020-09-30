package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCostume;

public class Wall
{
   private static final double wallThickness = 0.1;

   private final Sprite sprite;

   private final Point2D pointOne, pointTwo;
   private final LineSegment2D lineSegment;

   public Wall(Point2D pointOne, Point2D pointTwo)
   {
      this.pointOne = new Point2D(pointOne);
      this.pointTwo = new Point2D(pointTwo);

      this.lineSegment = new LineSegment2D(pointOne, pointTwo);
      ArrayList<Point2D> wallPoints = getWallPoints();
      
      sprite = createSprite(wallPoints);

//      ConvexPolygon collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.15, -0.15), new Vector(0.3, 0.3));
//      sprite.addCollisionPolygon(collisionPolygon);
   }

   private ArrayList<Point2D> getWallPoints()
   {
      Vector2D vector = new Vector2D(pointTwo);
      vector.sub(pointOne);
      vector.normalize();

      Vector2D perpendicular = new Vector2D(-vector.getY(), vector.getX());

      ArrayList<Point2D> points = new ArrayList<Point2D>();
      perpendicular.scale(wallThickness / 2.0);

      Point2D pointA = new Point2D(pointOne);
      pointA.add(perpendicular);
      points.add(pointA);

      Point2D pointB = new Point2D(pointOne);
      pointB.sub(perpendicular);
      points.add(pointB);

      Point2D pointC = new Point2D(pointTwo);
      pointC.sub(perpendicular);
      points.add(pointC);

      Point2D pointD = new Point2D(pointTwo);
      pointD.add(perpendicular);
      points.add(pointD);

      return points;
   }

   private Sprite createSprite(ArrayList<Point2D> wallPoints)
   {
      Sprite sprite = new Sprite("Wall");

      double xMin = Double.POSITIVE_INFINITY;
      double yMin = Double.POSITIVE_INFINITY;

      double xMax = Double.NEGATIVE_INFINITY;
      double yMax = Double.NEGATIVE_INFINITY;
      
      for (Point2D wallPoint : wallPoints)
      {
         if (wallPoint.getX() < xMin)
            xMin = wallPoint.getX();
         if (wallPoint.getY() < yMin)
            yMin = wallPoint.getY();
         
         if (wallPoint.getX() > xMax)
            xMax = wallPoint.getX();
         if (wallPoint.getY() > yMax)
            yMax = wallPoint.getY();
      }

      double deltaX = Math.abs(xMax - xMin);
      double deltaY = Math.abs(yMax - yMin);

      double scale = 100.0;
      int xPixels = (int) (deltaX * scale);
      int yPixels = (int) (deltaY * scale);

      BufferedImage bufferedImage = new BufferedImage(xPixels, yPixels, BufferedImage.TYPE_INT_ARGB);
      Graphics2D graphics2D = bufferedImage.createGraphics();

      graphics2D.setColor(Color.blue);

      int[] xPoints = new int[wallPoints.size()];
      int[] yPoints = new int[wallPoints.size()];
      int nPoints = wallPoints.size();
      
      for (int i=0; i<wallPoints.size(); i++)
      {
         Point2D wallPoint = wallPoints.get(i);
         xPoints[i] = (int) ((wallPoint.getX()-xMin) * scale);
         yPoints[i] = (int) ((wallPoint.getY()-yMin) * scale);
      }
      graphics2D.fillPolygon(xPoints, yPoints, nPoints);

      graphics2D.dispose();

      Image fxImage = SwingFXUtils.toFXImage(bufferedImage, null);
      SpriteCostume costume = new SpriteCostume(fxImage);
      costume.setXReferencePercent(0.0);
      costume.setYReferencePercent(0.0);
      sprite.addCostume(costume);

      sprite.setX(xMin);
      sprite.setY(yMin);
      
      sprite.setWidth(deltaX);
      sprite.setHeight(deltaY);

      return sprite;
   }

   public Sprite getSprite()
   {
      return sprite;
   }

   public Point2DReadOnly getPointOne()
   {
      return pointOne;
   }

   public Point2DReadOnly getPointTwo()
   {
      return pointTwo;
   }

   public LineSegment2DReadOnly getLineSegment()
   {
      return lineSegment;
   }

}
