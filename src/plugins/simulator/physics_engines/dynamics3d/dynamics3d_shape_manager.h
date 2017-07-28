/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_shape_manager.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_SHAPE_MANAGER_H
#define DYNAMICS3D_SHAPE_MANAGER_H

namespace argos {
   class CDynamics3DShapeManager;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionShapes/btBoxShape.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionShapes/btCylinderShape.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionShapes/btSphereShape.h>

#include <vector>

namespace argos {

   /**
    * Collision shape manager for the dynamics 3D physics engine.
    * <p>
    * This class offers all the basic functionality to manage
    * models composed of a single body. These models, while having
    * only a single body, can have multiple associated shapes.
    * </p>
    * <p>
    * Single-body objects are often passive objects such as boxes and
    * cylinders, and simple robots with no moving parts, such as the
    * e-puck.
    * </p>
    * <p>
    * To use this class, simply create a class that inherits from it.
    * The default implementations for Reset(), MoveTo(), etc. should
    * be already OK for your needs.
    * </p>
    * <p>
    * In the constructor of your class, be sure to call the method SetBody()
    * when you have created the body and associated the shapes to it. If this
    * method is not called, the default implementations will have segfaults.
    * </p>
    * <p>
    * This class retains ownership for the body and shapes you created once
    * you call SetBody(). In other words, you don't need to free these objects
    * because ~CDynamics3DSingleBodyObjectModel() does it for you.
    * </p>
    */
   class CDynamics3DShapeManager {
      
   public:
      
      static btBoxShape* RequestBox(const btVector3& c_half_extents);
      static void ReleaseBox(const btBoxShape* pc_release);
      static btCylinderShape* RequestCylinder(const btVector3& c_half_extents);
      static void ReleaseCylinder(const btCylinderShape* pc_release);
      static btSphereShape* RequestSphere(btScalar f_radius);
      static void ReleaseSphere(const btSphereShape* pc_release);
      
   private:
      /* boxes */
      struct SBoxResource {
         SBoxResource(const btVector3& c_half_extents);
         btVector3 HalfExtents;
         btBoxShape* Shape;
         size_t InUseCount;
      };
      static std::vector<SBoxResource> m_vecBoxResources;

      /* cylinders */
      struct SCylinderResource {
         SCylinderResource(const btVector3& c_half_extents);
         btVector3 HalfExtents;
         btCylinderShape* Shape;
         size_t InUseCount;
      };
      static std::vector<SCylinderResource> m_vecCylinderResources;

      /* spheres */
      struct SSphereResource {
         SSphereResource(btScalar f_radius);
         btScalar Radius;
         btSphereShape* Shape;
         size_t InUseCount;
      };
      static std::vector<SSphereResource> m_vecSphereResources;

   };

}

#endif
