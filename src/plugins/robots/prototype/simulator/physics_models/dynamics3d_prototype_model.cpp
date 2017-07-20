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

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>

namespace argos {

   /****************************************/
   /****************************************/

   // this needs to be registered for each link
   // m_vecMotionStates must be organised to reflect the anchor indices
   // TODO: note that we have a void UpdateOriginAnchor(SAnchor& s_anchor); in the base class
   // perhaps for this model this method is never registered or implemented? the registering of this method in the 2d dynamics engine occurs e.g. inside the footbot model

   void CDynamics3DPrototypeModel::UpdateLinkAnchor(SAnchor& s_anchor) {
      /* look up the correct motion state using the anchors index */
      const btDefaultMotionState& cMotionState = m_vecMotionStates[s_anchor.Index];
      /* retrieve the position and orientation */
      const btVector3& cPosition = (cMotionState.m_graphicsWorldTrans).getOrigin();
      const btQuaternion& cOrientation = (cMotionState.m_graphicsWorldTrans).getRotation();
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
      
      btVector3 cInertia;


      /* reserve memory */
      size_t unNumLinks = m_cLinkEquippedEntity.GetNumLinks();
      m_vecLinkShapes.resize(m_cLinkEquippedEntity.GetNumLinks());
      
      /* setup the reference link */
      CLinkEntity& cReferenceLink = c_entity.GetReferenceLink();

      btVector3 cReferenceLinkHalfExtents(cReferenceLink.GetExtents().GetX() * 0.5f,
                                 cReferenceLink.GetExtents().GetZ() * 0.5f,
                                 cReferenceLink.GetExtents().GetY() * 0.5f);

      btCollisionShape* pcReferenceLinkShape;
      
      /* TODO: factorize the link setup for the reference link and other links into base class */
      switch(cReferenceLink.GetGeometry()) {
         case CLinkEntity::EGeometry::BOX:
            pcReferenceLinkShape = CDynamics3DShapeManager::RequestBox(cReferenceLinkHalfExtents);
            break;
         default:
            THROW_ARGOSEXCEPTION("Not implemented");
            break;
      }

      // TODO: implement a check to determine if inertia is already defined (e.g. via URDF)
      Real fReferenceLinkMass = cReferenceLink.GetMass();
      
      pcReferenceLinkShape->calculateLocalInertia(fReferenceLinkMass, cInertia);


      SetMultiBody(m_cLinkEquippedEntity.GetNumLinks(), fReferenceLinkMass, cInertia);
     
      // iteratively build model using links

      // error conditions: detached link, loop (e.g. attempting to use the parent or an existing link as the child)
      // loop over links until the number of remaining links is zero (done) or the remaining links doesn't change (link detached from model)
      // in base class AddLink(parent, joint type, frame info), SetReferenceLinkLink
      // 

      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {

         // perhaps it's better to get an index?
         // use anchors directly?
         CLinkEntity& cParentLink = (*itJoint)->GetParentLink();
         CLinkEntity& cChildLink = (*itJoint)->GetChildLink();

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

         pcShape->calculateLocalInertia(fReferenceLinkMass, cInertia);

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

         
         switch((*itJoint)->GetType()) {
         case CJointEntity::EType::FIXED:
            // WARNING: HARDED CODED TEST, 0->1
            GetMultiBody().setupFixed(1,
                                      cChildLink.GetMass(),
                                      cInertia,
                                      0,
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
      }

      // create coliders
      // add as collision shapes to world

      /* Update the bodies inside the entity which have their positions driven by the physics engines */
      UpdateEntityStatus();
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

