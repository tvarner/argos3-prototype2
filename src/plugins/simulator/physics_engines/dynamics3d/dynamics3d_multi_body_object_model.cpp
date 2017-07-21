
#include "dynamics3d_multi_body_object_model.h"
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DMultiBodyObjectModel::CDynamics3DMultiBodyObjectModel(CDynamics3DEngine& c_engine,
                                                                    CComposableEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetComponent<CEmbodiedEntity>("body")),
      m_cEntity(c_entity),
      m_pcMultiBody(NULL) {}

   /****************************************/
   /****************************************/

   CDynamics3DMultiBodyObjectModel::~CDynamics3DMultiBodyObjectModel() {
   }

   /****************************************/
   /****************************************/

   void CDynamics3DMultiBodyObjectModel::MoveTo(const CVector3& c_position,
                                                const CQuaternion& c_orientation) {
      /* Transform coordinate systems and move the body */
      /* Update body */
      /* Update the bounding box */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DMultiBodyObjectModel::Reset() {
      /* Reset body position */
      /* setup the body */
      /* set the default surface friction */
      /* For reverse look up */
      /* Activate the body */
      /* Update bounding box */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DMultiBodyObjectModel::CalculateBoundingBox() {
      btVector3 cAabbMin(-.1,-.1,-.1);
      btVector3 cAabbMax(.1,.1,.1);
      /* Get the axis aligned bounding box for the current body */
      //m_pcShape->getAabb(m_cBody.getWorldTransform(), cAabbMin, cAabbMax);
      /* Write back the bounding box swapping the coordinate systems and the Y component */
      GetBoundingBox().MinCorner.Set(cAabbMin.getX(), -cAabbMax.getZ(), cAabbMin.getY());
      GetBoundingBox().MaxCorner.Set(cAabbMax.getX(), -cAabbMin.getZ(), cAabbMax.getY());
   }
  
   /****************************************/
   /****************************************/

   // TODO: move this function back to the model class? Yes.
   bool CDynamics3DMultiBodyObjectModel::IsCollidingWithSomething() const {
      /* get the collision dispatcher */
      const btCollisionDispatcher* pcCollisionDispatcher =
         GetEngine().GetCollisionDispatcher();
      /* for each manifold from the collision dispatcher */
      for(UInt32 i = 0; i < UInt32(pcCollisionDispatcher->getNumManifolds()); i++) {
         const btPersistentManifold* pcContactManifold =
            pcCollisionDispatcher->getManifoldByIndexInternal(i);
         const CDynamics3DModel* pcModelA =
            static_cast<const CDynamics3DModel*>(pcContactManifold->getBody0()->getUserPointer());
         const CDynamics3DModel* pcModelB =
            static_cast<const CDynamics3DModel*>(pcContactManifold->getBody1()->getUserPointer());
         /* ignore collisions of bodies that don't belong to a model (e.g. the ground) */
         if((pcModelA == NULL) || (pcModelB == NULL)) {
            continue;
         }
         /* check that the collision involves this model */
         if((pcModelA == this) || (pcModelB == this)) {
            /* One of the two bodies involved in the contact manifold belongs to this model,
               check for contact points with negative distance to indicate a collision */
            for(UInt32 j = 0; j < UInt32(pcContactManifold->getNumContacts()); j++) {  
               const btManifoldPoint& cManifoldPoint = pcContactManifold->getContactPoint(j);
               if (cManifoldPoint.getDistance() < 0.0f) {
                  return true;
               }
            }
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DMultiBodyObjectModel::SetBase(size_t un_num_links, btScalar f_mass, const btVector3& c_inertia) {
      /* setup the multi body model */
      m_pcMultiBody = new btMultiBody(un_num_links, f_mass, c_inertia, false, false);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DMultiBodyObjectModel::Finalize() {
      m_pcMultiBody->finalizeMultiDof();
      /* Add body to world */
      GetEngine().GetPhysicsWorld()->addMultiBody(m_pcMultiBody);
      /* Add colliders to world */

      /* Calculate the bounding box */
      CalculateBoundingBox();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DMultiBodyObjectModel::UpdateOriginAnchor(SAnchor& s_anchor) {
   }

   /****************************************/
   /****************************************/

}
