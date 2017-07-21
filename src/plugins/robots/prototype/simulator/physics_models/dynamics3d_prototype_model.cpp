/**
 * @file <argos3/plugins/robots/prototype/simulator/physics_models/dynamics3d_prototype_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_prototype_model.h"

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_multi_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_shape_manager.h>

#include <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.h>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>

namespace argos {

   /****************************************/
   /****************************************/

   // this needs to be registered for each link
   // m_vecMotionStates must be organised to reflect the anchor indices
   // TODO: note that we have a void UpdateOriginAnchor(SAnchor& s_anchor); in the base class
   // perhaps for this model this method is never registered or implemented? the registering of this method in the 2d dynamics engine occurs e.g. inside the footbot model

   //btMultibodyLink& btMultibody::getLink(int index)
   //btMultibodyLink::m_cachedWorldTransform

   // TODO: This could be moved to the base class - NO! only the prototype model needs to have all its links synced
   // TODO: note that the base needs a seperate anchor method
   void CDynamics3DPrototypeModel::UpdateLinkAnchor(SAnchor& s_anchor) {
      /* look up the links world transform using the anchors index */
      const btTransform& cLinkTransform = GetMultiBody().getLink(s_anchor.Index - 1).m_cachedWorldTransform;
      /* retrieve the position and orientation */
      const btVector3& cPosition = cLinkTransform.getOrigin();
      const btQuaternion& cOrientation = cLinkTransform.getRotation();
      /* swap coordinate system and set position */
      s_anchor.Position.Set(cPosition.getX(), -cPosition.getZ(), cPosition.getY());
      /* swap coordinate system and set orientation */
      s_anchor.Orientation.Set(cOrientation.getW(),
                               cOrientation.getX(),
                              -cOrientation.getZ(),
                               cOrientation.getY());
   }

   void CDynamics3DPrototypeModel::UpdateBaseAnchor(SAnchor& s_anchor) {
      /* look up the base's world transform */
      const btTransform& cLinkTransform = GetMultiBody().getBaseWorldTransform();
      /* retrieve the position and orientation */
      const btVector3& cPosition = cLinkTransform.getOrigin();
      const btQuaternion& cOrientation = cLinkTransform.getRotation();
      /* swap coordinate system and set position */
      s_anchor.Position.Set(cPosition.getX(), -cPosition.getZ(), cPosition.getY());
      /* swap coordinate system and set orientation */
      s_anchor.Orientation.Set(cOrientation.getW(),
                               cOrientation.getX(),
                              -cOrientation.getZ(),
                               cOrientation.getY());
   }

   
   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::CDynamics3DPrototypeModel(CDynamics3DEngine& c_engine,
                                                        CPrototypeEntity& c_entity) :
      CDynamics3DMultiBodyObjectModel(c_engine, c_entity),
      m_cPrototypeEntity(c_entity),
      m_cLinkEquippedEntity(c_entity.GetLinkEquippedEntity()),
      m_cJointEquippedEntity(c_entity.GetJointEquippedEntity()) {


      LOGERR << "m_cLinkEquippedEntity.GetNumLinks() = " << m_cLinkEquippedEntity.GetNumLinks() << std::endl;
      
      btVector3 cInertia;


      /* reserve memory */
      size_t unNumLinks = m_cLinkEquippedEntity.GetNumLinks();
      m_vecLinkShapes.resize(m_cLinkEquippedEntity.GetNumLinks());
      
      /* use the reference link as the base of the robot */
      CLinkEntity& cBase = c_entity.GetReferenceLink();

      LOGERR << "c_entity.GetReferenceLink().GetId() = " << c_entity.GetReferenceLink().GetId() << std::endl;

      btVector3 cBaseHalfExtents(cBase.GetExtents().GetX() * 0.5f,
                                 cBase.GetExtents().GetZ() * 0.5f,
                                 cBase.GetExtents().GetY() * 0.5f);


      /* TODO: perhaps use a struct similar to this in the base class
       struct SLink {
         SLink(btMultiBody& c_multi_body, ) :
            Collider(&c_multi_body, -1) {}
         btMultiBodyLinkCollider Collider;
       };
      */
     
      /* TODO: factorize the link setup for the reference link and other links into base class */
      btCollisionShape* pcBaseShape;

      switch(cBase.GetGeometry()) {
         case CLinkEntity::EGeometry::BOX:
            pcBaseShape = CDynamics3DShapeManager::RequestBox(cBaseHalfExtents);
            break;
         default:
            THROW_ARGOSEXCEPTION("Not implemented");
            break;
      }

      btMultiBodyLinkCollider* pcBaseCollider = new btMultiBodyLinkCollider(&GetMultiBody(), -1);
      pcBaseCollider->setCollisionShape(pcBaseShape);
		pcBaseCollider->setFriction(0.5);
      pcBaseCollider->setUserPointer(this);

      const CVector3& cBasePosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      const CQuaternion& cBaseOrientation = GetEmbodiedEntity().GetOriginAnchor().Orientation;

      btTransform tb(btQuaternion(cBaseOrientation.GetX(),
                               cBaseOrientation.GetZ(), 
                              -cBaseOrientation.GetY(),
                               cBaseOrientation.GetW()),
                     btVector3(cBasePosition.GetX(),
                               cBasePosition.GetZ(),
                              -cBasePosition.GetY()));

      pcBaseCollider->setWorldTransform(tb);

      // TODO: implement a check to determine if inertia is already defined (e.g. via URDF)
      Real fBaseMass = cBase.GetMass();
      
      pcBaseShape->calculateLocalInertia(fBaseMass, cInertia);
     
      SetBase(m_cLinkEquippedEntity.GetNumLinks() - 1, fBaseMass, cInertia);
 
      // iteratively build model using links

      // error conditions: detached link, loop (e.g. attempting to use the parent or an existing link as the child)
      // loop over links until the number of remaining links is zero (done) or the remaining links doesn't change (link detached from model)
      // in base class AddLink(parent, joint type, frame info), SetBaseLink
      // 

      btMultiBodyLinkCollider* pcCollider;

      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {

         // perhaps it's better to get an index?
         // use anchors directly?
         CLinkEntity& cParentLink = (*itJoint)->GetParentLink();
         CLinkEntity& cChildLink = (*itJoint)->GetChildLink();

         std::cerr << (*itJoint)->GetId() << ".parent_link = " << cParentLink.GetId() << ", anchor = " << cParentLink.GetAnchor().Index << std::endl;
         std::cerr << (*itJoint)->GetId() << ".child_link = " << cChildLink.GetId() << ", anchor = " << cChildLink.GetAnchor().Index << std::endl;

         btVector3 cHalfExtents(cChildLink.GetExtents().GetX() * 0.5f,
                                cChildLink.GetExtents().GetZ() * 0.5f,
                                cChildLink.GetExtents().GetY() * 0.5f);
         
         btCollisionShape* pcShape;

         switch(cChildLink.GetGeometry()) {
         case CLinkEntity::EGeometry::BOX:
            pcShape = CDynamics3DShapeManager::RequestBox(cHalfExtents);
            break;
         default:
            THROW_ARGOSEXCEPTION("Link geometry not implemented");
            break;
         }

         pcCollider = new btMultiBodyLinkCollider(&GetMultiBody(), cChildLink.GetAnchor().Index - 1);
         pcCollider->setCollisionShape(pcBaseShape);
   		pcCollider->setFriction(0.5);
         pcCollider->setUserPointer(this);

         const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
         const CQuaternion& cOrientation = GetEmbodiedEntity().GetOriginAnchor().Orientation;

         btTransform tc(btQuaternion(cBaseOrientation.GetX(),
                               cOrientation.GetZ(), 
                              -cOrientation.GetY(),
                               cOrientation.GetW()),
                     btVector3(cPosition.GetX(),
                               cPosition.GetZ(),
                              -cPosition.GetY()));

         pcCollider->setWorldTransform(tc);

         c_engine.GetPhysicsWorld()->addCollisionObject(pcCollider);

         pcShape->calculateLocalInertia(fBaseMass, cInertia);

         /*
           btTransform offsetInA,offsetInB;
            offsetInA = parentLocalInertialFrame.inverse()*parent2joint;
            offsetInB = localInertialFrame.inverse();
            btQuaternion parentRotToThis = offsetInB.getRotation() * offsetInA.inverse().getRotation();
            m_bulletMultiBody->setupFixed(mbLinkIndex, mass, localInertiaDiagonal, mbParentIndex,
                                          parentRotToThis, offsetInA.getOrigin(),-offsetInB.getOrigin());

          */

         // since we are in init, perhaps these are taken from the offset positions and orientations in the anchor
         // we are only calculating the relative rotation and translation between the links
         // leave note somewhere that the joint axis always coincides with the child axis
         btQuaternion cParentLinkOrientation(cParentLink.GetAnchor().OffsetOrientation.GetX(),
                                             cParentLink.GetAnchor().OffsetOrientation.GetZ(), 
                                             -cParentLink.GetAnchor().OffsetOrientation.GetY(),
                                             cParentLink.GetAnchor().OffsetOrientation.GetW());
         btQuaternion cChildLinkOrientation(cChildLink.GetAnchor().OffsetOrientation.GetX(),
                                             cChildLink.GetAnchor().OffsetOrientation.GetZ(), 
                                             -cChildLink.GetAnchor().OffsetOrientation.GetY(),
                                             cChildLink.GetAnchor().OffsetOrientation.GetW());

         CVector3 cParentToChildOffset = cParentLink.GetAnchor().OffsetPosition - cChildLink.GetAnchor().OffsetPosition;


         /*
           - parentComToThisPivotOffset is the pivot (translation) from parent body center of mass (COM) to the constraint frame.
           - thisPivotToThisComOffset is the pivot from child center of mass to the constraint frame.
          */

         // TODO: Note that I set the link index here
         // TODO: possible segfault from here, we need to assume that the base or ref link is the anchor with index zero and subtract 1 from all indices since the base link is not included in this array
         // TODO: this seems correct, since if our anchor index is 0, then our base link is -1 which is consistent with the representation in bullet
         switch((*itJoint)->GetType()) {
         case CJointEntity::EType::FIXED:
            GetMultiBody().setupFixed(cChildLink.GetAnchor().Index - 1,
                                      cChildLink.GetMass(),
                                      cInertia,
                                      cParentLink.GetAnchor().Index - 1,
                                      cParentLinkOrientation.inverse() * cChildLinkOrientation,
                                      btVector3(cParentToChildOffset.GetX(),
                                                cParentToChildOffset.GetZ(),
                                                -cParentToChildOffset.GetY()),  
                                      btVector3(0,0,0));
            break;
         default:
            THROW_ARGOSEXCEPTION("Joint type not implemented");
            break;
         }

         RegisterAnchorMethod(cChildLink.GetAnchor(),
                           &CDynamics3DPrototypeModel::UpdateLinkAnchor);
      }

      // setup
      Finalize();

      /* Register the origin anchor update method */
      RegisterAnchorMethod(GetEmbodiedEntity().GetOriginAnchor(),
                           &CDynamics3DPrototypeModel::UpdateBaseAnchor);

      

      // Colliders TODO: loop over an array in the base class
      c_engine.GetPhysicsWorld()->addCollisionObject(pcBaseCollider);
      c_engine.GetPhysicsWorld()->addCollisionObject(pcCollider);
   }

   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::~CDynamics3DPrototypeModel() {
      // clean up
      // release collision shapes to the shape manager
      // deallcate any memory
   }

   /****************************************/
   /****************************************/

   // TODO default implementation takes care of calling the anchor methods. Do we need to write back joint settings
   // Perhaps it is cleaner to calculate the joint settings in the space using the anchors
   // We could register specific joints for which we want to calculate the translations/orientations and only calculate on demand
   // This could be done in the space
   /*
   void CDynamics3DPrototypeModel::UpdateEntityStatus() {
   }
   */

   /****************************************/
   /****************************************/

   void CDynamics3DPrototypeModel::UpdateFromEntityStatus() {
      // TODO: Read in joint actuator settings
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CPrototypeEntity, CDynamics3DPrototypeModel);

   /****************************************/
   /****************************************/

}

