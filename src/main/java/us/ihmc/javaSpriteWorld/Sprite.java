package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

import us.ihmc.javaSpriteWorld.geometry.AxisAlignedBoundingBox2D;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;

public class Sprite
{
   private static final double radiansToDegrees = 180.0 / Math.PI;

   private final String name;

   private ArrayList<SpriteCostume> costumes = new ArrayList<>();
   private ArrayList<SpriteMouseListener> spriteMouseListeners = new ArrayList<>();

   private double width = 0.1, height = 0.1;

   private double x = 0.0, y = 0.0;
   private boolean reflectX = false;
   private boolean reflectY = false;

   private int costumeNumber = -1;
   private double rotationInRadians = 0.0;
   private boolean visible = true;

   private ArrayList<ConvexPolygon> defaultCollisionPolygons, transformedCollisionPolygons;
   private final AxisAlignedBoundingBox2D transformedBoundingBox = new AxisAlignedBoundingBox2D();

   private boolean collisionPolygonsDirty = true;

   public Sprite(String name)
   {
      this.name = name;
   }

   public void addCollisionPolygonToMatchCostumeRectangle(int costumeNumber)
   {
      ArrayList<Point> points = new ArrayList<Point>();

      SpriteCostume spriteCostume = costumes.get(costumeNumber);
      double xReferencePercent = spriteCostume.getXReferencePercent();
      double yReferencePercent = spriteCostume.getYReferencePercent();

      points.add(new Point(-width * xReferencePercent, -height * yReferencePercent));
      points.add(new Point(width * (1.0 - xReferencePercent), -height * yReferencePercent));
      points.add(new Point(width * (1.0 - xReferencePercent), height * (1.0 - yReferencePercent)));
      points.add(new Point(-width * xReferencePercent, height * (1.0 - yReferencePercent)));
      ConvexPolygon collisionPolygon = new ConvexPolygon(points);

      addCollisionPolygon(collisionPolygon);
   }

   public void addCollisionPolygon(ConvexPolygon collisionPolygon)
   {
      if (defaultCollisionPolygons == null)
      {
         defaultCollisionPolygons = new ArrayList<>();
         transformedCollisionPolygons = new ArrayList<>();
      }

      defaultCollisionPolygons.add(collisionPolygon);
      transformedCollisionPolygons.add(new ConvexPolygon(collisionPolygon));

      collisionPolygonsDirty = true;
   }

   public void getAxisAlignedBoundingBox(AxisAlignedBoundingBox2D axisAlignedBoundingBoxToPack)
   {
      axisAlignedBoundingBoxToPack.setToNaN();

      if (defaultCollisionPolygons == null)
         return;

      if (collisionPolygonsDirty)
      {
         computeTransformedCollisionPolygonsAndBoundingBox();
      }

      axisAlignedBoundingBoxToPack.set(this.transformedBoundingBox);
   }

   public void getCollisionPolygons(ArrayList<ConvexPolygon> collisionPolygonsToPack)
   {
      collisionPolygonsToPack.clear();

      if (defaultCollisionPolygons == null)
         return;

      if (collisionPolygonsDirty)
      {
         computeTransformedCollisionPolygonsAndBoundingBox();
      }

      collisionPolygonsToPack.addAll(transformedCollisionPolygons);
   }

   private void computeTransformedCollisionPolygonsAndBoundingBox()
   {
      transformedBoundingBox.setToNaN();
      if (defaultCollisionPolygons == null)
         return;

      for (int i = 0; i < defaultCollisionPolygons.size(); i++)
      {
         ConvexPolygon convexPolygon = defaultCollisionPolygons.get(i);
         ConvexPolygon transformedPolygon = transformedCollisionPolygons.get(i);

         transformedPolygon.set(convexPolygon);
         transformedPolygon.transform(this.reflectX, this.reflectY, this.x, this.y, this.rotationInRadians);

         transformedPolygon.growBoundingBoxToIncludeThisPolygon(transformedBoundingBox);
      }

      collisionPolygonsDirty = false;
   }

   public String getName()
   {
      return name;
   }

   public synchronized int addCostume(String filename)
   {
      SpriteCostume costume = SpriteCostume.createFromFile(filename);
      if (costume != null)
         return addCostume(costume);

      return -1;
   }

   public synchronized void clearCostumes()
   {
      this.costumeNumber = -1;
      this.costumes.clear();
   }

   public synchronized int addCostume(SpriteCostume spriteCostume)
   {
      costumes.add(spriteCostume);
      int costumeNumber = costumes.size() - 1;
      if (costumeNumber == 0)
         this.costumeNumber = 0;

      return costumeNumber;
   }

   public synchronized void switchToCostume(int costumeNumber)
   {
      this.costumeNumber = costumeNumber;
   }

   public synchronized SpriteCostume getCostume()
   {
      if ((costumeNumber >= 0) && (costumeNumber < costumes.size()))
         return costumes.get(costumeNumber);

      else
         return null;
   }

   public int nextCostume()
   {
      costumeNumber++;
      if (costumeNumber >= costumes.size())
         costumeNumber = 0;

      return costumeNumber;
   }

   public int previousCostume()
   {
      costumeNumber--;
      if (costumeNumber < 0)
         costumeNumber = costumes.size() - 1;

      return costumeNumber;
   }

   public int getNumberOfCostumes()
   {
      return costumes.size();
   }

   public synchronized void setCostume(int costumeNumber)
   {
      this.costumeNumber = costumeNumber;
   }

   public ArrayList<SpriteMouseListener> getSpriteMouseListeners()
   {
      return spriteMouseListeners;
   }

   public void attachSpriteMouseListener(SpriteMouseListener spriteMouseListener)
   {
      spriteMouseListeners.add(spriteMouseListener);
   }

   public synchronized double getRotationInRadians()
   {
      return rotationInRadians;
   }

   public synchronized double getRotationInDegrees()
   {
      return rotationInRadians * radiansToDegrees;
   }

   public synchronized void setRotationInRadians(double rotationInRadians)
   {
      this.rotationInRadians = rotationInRadians;
      collisionPolygonsDirty = true;
   }

   public synchronized void setRotationInDegrees(double rotationInDegrees)
   {
      this.rotationInRadians = rotationInDegrees / radiansToDegrees;
      collisionPolygonsDirty = true;
   }

   public synchronized void addRotationInRadians(double incrementalRotationInRadians)
   {
      this.rotationInRadians += incrementalRotationInRadians;
      collisionPolygonsDirty = true;
   }

   public synchronized void addRotationInDegrees(double incrementalRotationInDegrees)
   {
      this.rotationInRadians += incrementalRotationInDegrees / radiansToDegrees;
      collisionPolygonsDirty = true;
   }

   public synchronized double getX()
   {
      return x;
   }

   public synchronized void setX(double x)
   {
      this.x = x;
      collisionPolygonsDirty = true;
   }

   public synchronized void addX(double incrementalX)
   {
      this.x += incrementalX;
      collisionPolygonsDirty = true;
   }

   public synchronized double getY()
   {
      return y;
   }

   public synchronized void setY(double y)
   {
      this.y = y;
      collisionPolygonsDirty = true;
   }

   public synchronized void addY(double incrementalY)
   {
      this.y += incrementalY;
      collisionPolygonsDirty = true;
   }

   public void setWidth(double width)
   {
      this.width = width;
      collisionPolygonsDirty = true;
   }

   public void setHeight(double height)
   {
      this.height = height;
      collisionPolygonsDirty = true;
   }

   public void setWidthPreserveScale(double width, int costumeNumber)
   {
      this.width = width;
      this.height = width * costumes.get(costumeNumber).getHeightToWidthRatio();
      collisionPolygonsDirty = true;
   }

   public void setHeightPreserveScale(double height, int costumeNumber)
   {
      this.height = height;
      this.width = height / costumes.get(costumeNumber).getHeightToWidthRatio();
      collisionPolygonsDirty = true;
   }

   public double getWidth()
   {
      return width;
   }

   public double getHeight()
   {
      return height;
   }

   public void hide()
   {
      this.visible = false;
   }

   public void show()
   {
      this.visible = true;
   }

   public void setVisible(boolean visible)
   {
      this.visible = visible;
   }

   public boolean isVisible()
   {
      return visible;
   }

   public SpriteCostume getCostume(int i)
   {
      return costumes.get(i);
   }

   public boolean getReflectX()
   {
      return reflectX;
   }

   public void setReflectX(boolean reflectX)
   {
      this.reflectX = reflectX;
      collisionPolygonsDirty = true;
   }

   public boolean getReflectY()
   {
      return reflectY;
   }

   public void setReflectY(boolean reflectY)
   {
      this.reflectY = reflectY;
      collisionPolygonsDirty = true;
   }

   public boolean isClickPointInside(double worldX, double worldY)
   {
      if (defaultCollisionPolygons == null)
         return isClickPointInsideSimple(worldX, worldY);

      if (collisionPolygonsDirty)
         computeTransformedCollisionPolygonsAndBoundingBox();

      if (!transformedBoundingBox.isPointInside(worldX, worldY))
         return false;
      for (int i = 0; i < transformedCollisionPolygons.size(); i++)
      {
         ConvexPolygon convexPolygon = transformedCollisionPolygons.get(i);
         if (convexPolygon.isPointInsideBoundingBox(worldX, worldY))
            return true;
      }

      return false;
   }

   private boolean isClickPointInsideSimple(double worldX, double worldY)
   {
      SpriteCostume costume = this.getCostume();
      double xReferencePercent = costume.getXReferencePercent();
      double yReferencePercent = costume.getYReferencePercent();

      double spriteWidth = getWidth();
      double spriteHeight = getHeight();

      double spriteX = getX();
      double spriteY = getY();
      double spriteRotation = getRotationInRadians();

      double xClickInSpriteTranslatedFrame = worldX - spriteX;
      double yClickInSpriteTranslatedFrame = worldY - spriteY;
      
      double cosine = Math.cos(spriteRotation);
      double sine = Math.sin(spriteRotation);
      
      double xClickInSpriteFrame = cosine * xClickInSpriteTranslatedFrame + sine * yClickInSpriteTranslatedFrame;
      double yClickInSpriteFrame = -sine * xClickInSpriteTranslatedFrame + cosine * yClickInSpriteTranslatedFrame;
      
      double minXToCheck = - xReferencePercent * spriteWidth;
      double maxXToCheck = (1.0 - xReferencePercent) * spriteWidth;
      double minYToCheck = - yReferencePercent * spriteHeight;
      double maxYToCheck = (1.0 - yReferencePercent) * spriteHeight;

      if ((xClickInSpriteFrame > minXToCheck) && (xClickInSpriteFrame < maxXToCheck) && (yClickInSpriteFrame > minYToCheck) && (yClickInSpriteFrame < maxYToCheck))
      {
         return true;
      }

      return false;
   }
}
