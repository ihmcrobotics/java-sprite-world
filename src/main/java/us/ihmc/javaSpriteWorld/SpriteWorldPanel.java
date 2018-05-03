package us.ihmc.javaSpriteWorld;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.util.List;

import javax.swing.JPanel;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;

public class SpriteWorldPanel extends JPanel implements MouseMotionListener, MouseListener
{
   private static final long serialVersionUID = 4076112028926697537L;
   
   private final AffineTransform scaleForViewerPixelSize = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform scaleForWorldSize = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform translateForTopLeftCorner = AffineTransform.getTranslateInstance(0.0, 0.0);
   private final AffineTransform translateForSpritePosition = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform spriteRotation = AffineTransform.getRotateInstance(0.0);
   private final AffineTransform spriteReflection = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform scaleForSpriteSizeToImageSize = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform translateForImageCenter = AffineTransform.getTranslateInstance(0.0, 0.0);
   
   private final AffineTransform combinedTransform = new AffineTransform();

   private SpriteWorld spriteWorld;
   private GenericMouseEventHandler mouseEventHandler;

   public SpriteWorldPanel(SpriteWorld spriteWorld)
   {
      setSpriteWorld(spriteWorld);

      this.addMouseListener(this);
      this.addMouseMotionListener(this);
   }

   public void setSpriteWorld(SpriteWorld spriteWorld) 
   {
      this.spriteWorld = spriteWorld;
      this.mouseEventHandler = new GenericMouseEventHandler(spriteWorld);
   }

   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      Graphics2D graphics = (Graphics2D) g;
      
      if (spriteWorld == null) return;
      
      SpriteStage stage = spriteWorld.getSpriteStage();

      if (stage != null)
      {
         paintSprite(stage, graphics);
      }

      List<Sprite> sprites = spriteWorld.getSprites();

      for (int i=0; i<sprites.size(); i++)
      {
         Sprite sprite = sprites.get(i);
         paintSprite(sprite, graphics);
      }
   }
   
   private void paintSprite(Sprite sprite, Graphics2D graphics)
   {
      if (sprite.isHidden()) return;
      
      double panelWidth = getWidth();
      double panelHeight = getHeight();
      
      SpriteCostume currentCostume = sprite.getCostume();
      Image fxImage = currentCostume.getImage();
      
      BufferedImage image = SwingFXUtils.fromFXImage(fxImage, null);
      
      if (image == null) return;
      
      double leftBorderX = spriteWorld.getLeftBorderX();
      double topBorderY = spriteWorld.getTopBorderY();
      double rightBorderX = spriteWorld.getRightBorderX();
      double bottomBorderY = spriteWorld.getBottomBorderY();

      double imageWidthPixels = image.getWidth();
      double imageHeightPixels = image.getHeight();

      double xReferencePercent = currentCostume.getXReferencePercent();
      double yReferencePercent = currentCostume.getYReferencePercent();

      double imageCenterX = imageWidthPixels * xReferencePercent;
      double imageCenterY = imageHeightPixels * yReferencePercent;

      scaleForViewerPixelSize.setToScale((double) panelWidth, ((double) panelHeight));
      
      scaleForWorldSize.setToScale(1.0/(rightBorderX - leftBorderX), 1.0/(bottomBorderY - topBorderY));
      translateForTopLeftCorner.setToTranslation(-leftBorderX, -topBorderY);
      
      translateForSpritePosition.setToTranslation(sprite.getX(), sprite.getY());
     
      spriteRotation.setToRotation(sprite.getRotationInRadians());
      
      double reflectX = 1.0;
      double reflectY = 1.0;
      
      if (sprite.getReflectX()) reflectX = -1.0;
      if (sprite.getReflectY()) reflectY = -1.0;
      
      spriteReflection.setToScale(reflectX, reflectY);
      
      scaleForSpriteSizeToImageSize.setToScale(sprite.getWidth() / imageWidthPixels, sprite.getHeight() / imageHeightPixels);
      translateForImageCenter.setToTranslation(-imageCenterX, -imageCenterY);
      
      combinedTransform.setToIdentity();

      combinedTransform.concatenate(scaleForViewerPixelSize);
      combinedTransform.concatenate(scaleForWorldSize);
      combinedTransform.concatenate(translateForTopLeftCorner);
      combinedTransform.concatenate(translateForSpritePosition);
      combinedTransform.concatenate(spriteRotation);
      combinedTransform.concatenate(spriteReflection);
      combinedTransform.concatenate(scaleForSpriteSizeToImageSize);
      combinedTransform.concatenate(translateForImageCenter);
      
      graphics.drawImage(image, combinedTransform, null);
   }

   public double convertFromPanelPixelsToWorldX(int panelPixels)
   {
	   double panelWidth = getWidth();

	   double leftBorderX = spriteWorld.getLeftBorderX();
	   double rightBorderX = spriteWorld.getRightBorderX();
	   
	   return leftBorderX + ((double) panelPixels) / panelWidth * (rightBorderX - leftBorderX);
   }
   
   public double convertFromPanelPixelsToWorldY(int panelPixels)
   {
	   double panelHeight = getHeight();

	   double topBorderY = spriteWorld.getTopBorderY();
	   double bottomBorderY = spriteWorld.getBottomBorderY();
	   
	   return topBorderY + ((double) panelPixels) / panelHeight * (bottomBorderY - topBorderY);
   }

   @Override
   public void mouseEntered(MouseEvent mouseEvent)
   {
      double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
      double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
      mouseEventHandler.mouseEntered(xWorld, yWorld);  
   }
   
   @Override
   public void mouseMoved(MouseEvent mouseEvent)
   {
      double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
      double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
      mouseEventHandler.mouseMoved(xWorld, yWorld);
   }
   
   @Override
   public void mouseExited(MouseEvent mouseEvent)
   {
      double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
      double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
      mouseEventHandler.mouseExited(xWorld, yWorld);
   }

   @Override
   public void mousePressed(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   mouseEventHandler.mousePressed(xWorld, yWorld);
   }

   @Override
   public void mouseDragged(MouseEvent mouseEvent) 
   {     
      double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
      double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
      mouseEventHandler.mouseDragged(xWorld, yWorld);
   }
   
   @Override
   public void mouseReleased(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   mouseEventHandler.mouseReleased(xWorld, yWorld);
   }
   
   @Override
   public void mouseClicked(MouseEvent mouseEvent) 
   {
      double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
      double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
      int clickCount = mouseEvent.getClickCount();
      mouseEventHandler.mouseClicked(xWorld, yWorld, clickCount);
   }

}