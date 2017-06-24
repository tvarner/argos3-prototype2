/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_single_body_object_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_SINGLE_BODY_OBJECT_MODEL_H
#define DYNAMICS3D_SINGLE_BODY_OBJECT_MODEL_H

namespace argos {
   class CDynamics3DSingleBodyObjectModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>

namespace argos {

   /**
    * Base class for object models with a single body.
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
   class CDynamics3DSingleBodyObjectModel : public CDynamics3DModel {

   public:

      /**
       * Class constructor.
       * @param c_engine The dynamics 2D engine that manages this model.
       * @param c_entity The composable entity associated to this model.
       * @throws CARGoSException if c_entity does not contain an embodied entity.
       */
      CDynamics3DSingleBodyObjectModel(CDynamics3DEngine& c_engine,
                                       CComposableEntity& c_entity);

      /**
       * Class destructor.
       * Disposes of the object body and its shapes.
       */
      virtual ~CDynamics3DSingleBodyObjectModel();

      /**
       * Returns the associated composable entity as a non-const reference.
       * @returns The associated composable entity as a non-const reference.
       */
      inline CComposableEntity& GetComposableEntity() {
         return m_cEntity;
      }

      /**
       * Returns the associated composable entity as a const reference.
       * @returns The associated composable entity as a const reference.
       */
      inline const CComposableEntity& GetComposableEntity() const {
         return m_cEntity;
      }

      /**
       * Returns the body as non-const pointer.
       * @returns The body as non-const pointer.
       */
      inline btRigidBody* GetBody() {
         return m_pcBody;
      }

      /**
       * Returns the body as const pointer.
       * @returns The body as const pointer.
       */
      inline const btRigidBody* GetBody() const {
         return m_pcBody;
      }

      virtual void Reset();

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation);

      virtual void CalculateBoundingBox();

      virtual void UpdateFromEntityStatus()  = 0;

      virtual bool IsCollidingWithSomething() const;

      /**
       * Sets the body and registers the default origin anchor method.
       * <p>
       * You must call this method for this class' methods to work.
       * </p>
       * <p>
       * This class retains ownership of the passed body, so you don't
       * need to explicitly delete anything.
       * </p>
       * <p>
       * Internally, this method also sets <tt>pt_body->data</tt> to
       * point to <tt>this</tt>, which is a requirement for ray
       * cast queries to work properly.
       * </p>
       * <p>
       * Finally, this method also calculates the initial bounding
       * box of the object.
       * </p>
       * <p>
       * @param pt_body The object body.
       */
      virtual void SetBody();

      /**
       * Updates the origin anchor associated to the embodied entity.
       */
      void UpdateOriginAnchor(SAnchor& s_anchor);

   private:

      CComposableEntity&    m_cEntity;
      btRigidBody*          m_pcBody;
      btDefaultMotionState* m_pcMotionState;

   protected:
      btCollisionShape*     m_pcShape;
      btVector3             m_cInertia;
      btScalar              m_fMass;
      btTransform           m_cGeometricOffset;
      btTransform           m_cPositionalOffset;
   };

}

#endif
