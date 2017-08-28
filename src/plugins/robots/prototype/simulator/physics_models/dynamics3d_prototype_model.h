/**
 * @file <argos3/plugins/robots/prototype/simulator/physics_models/dynamics3d_prototype_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_PROTOTYPE_MODEL_H
#define DYNAMICS3D_PROTOTYPE_MODEL_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_multi_body_object_model.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>

// remove me
#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>


namespace argos {

   class CDynamics3DPrototypeModel : public CDynamics3DMultiBodyObjectModel {

   public:

      CDynamics3DPrototypeModel(CDynamics3DEngine& c_engine,
                                CPrototypeEntity& c_entity);
      virtual ~CDynamics3DPrototypeModel();
           
      virtual void UpdateFromEntityStatus();

      void UpdateLinkAnchor(SAnchor& s_anchor);

      void UpdateBaseAnchor(SAnchor& s_anchor);

      struct SLink {
         SLink(btCollisionShape* pc_shape,
               btScalar f_mass,
               const SAnchor& s_anchor,
               const btVector3& c_inertia,
               const btTransform& c_transform,
               const btTransform& c_center_of_mass_offset) :
            Anchor(s_anchor),
            Shape(pc_shape),
            Collider(NULL),
            Mass(f_mass),
            Inertia(c_inertia),
            Transform(c_transform),
            CenterOfMassOffset(c_center_of_mass_offset) {}

         const SAnchor& Anchor;
         btCollisionShape* Shape;
         btMultiBodyLinkCollider* Collider;
         btScalar Mass;
         btVector3 Inertia;
         btTransform Transform;
         btTransform CenterOfMassOffset;

         void Init(CDynamics3DPrototypeModel* pc_model, SInt32 n_link_id) {
            Collider = new btMultiBodyLinkCollider(&(pc_model->GetMultiBody()), n_link_id);
            Collider->setCollisionShape(Shape);
            Collider->setWorldTransform(Transform * CenterOfMassOffset.inverse());
            Collider->setFriction(0.5);
            Collider->setUserPointer(pc_model);
            std::cerr << "init: " << n_link_id << std::endl;
            if(n_link_id == -1) {
               pc_model->GetMultiBody().setBaseCollider(Collider);
               pc_model->GetMultiBody().setBaseWorldTransform(Transform * CenterOfMassOffset.inverse());
            }
            else {
               pc_model->GetMultiBody().getLink(n_link_id).m_collider = Collider;
            }
         }
      };

   private:
      CLink& AddLink(const CLinkEntity& c_link);

      CPrototypeEntity&       m_cPrototypeEntity;
      CLinkEquippedEntity&    m_cLinkEquippedEntity;
      CJointEquippedEntity&   m_cJointEquippedEntity;

      std::vector<CLink>  m_vecLinks;
   };
}

#endif
