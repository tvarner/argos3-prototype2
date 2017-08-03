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

   std::ostream& operator<<(std::ostream& stream, const btVector3& vec) {
      stream << vec.getX() << ", " << vec.getY() << ", " << vec.getZ();
      return stream;
   }

   std::ostream& operator<<(std::ostream& stream, const btTransform& tr) {
      stream << "tr: " << tr.getOrigin().getX() << ", " << tr.getOrigin().getY() << ", " << tr.getOrigin().getZ() << std::endl;
      return stream;
   }


   /****************************************/
   /****************************************/

   // this needs to be registered for each link
   // m_vecMotionStates must be organised to reflect the anchor indices
   // TODO: note that we have a void UpdateOriginAnchor(SAnchor& s_anchor); in the base class
   // perhaps for this model this method is never registered or implemented? the registering of this method in the 2d dynamics engine occurs e.g. inside the footbot model

   //btMultibodyLink& btMultibody::getLink(int index)
   //btMultibodyLink::m_cachedWorldTransform

   // TODO: This could be moved to the base class - //NO! only the prototype model needs to have all its links synced// YES!? perhaps add all links and only enable the ones that need to be synced?
   // TODO: note that the base needs a seperate anchor method
   //
   void CDynamics3DPrototypeModel::UpdateLinkAnchor(SAnchor& s_anchor) {
      std::cerr << "UpdateLinkAnchor: " << s_anchor.Id << std::endl;
      /* look up the links world transform using the anchors index (offset by two) */
      const btTransform& cLinkTransform =
         GetMultiBody().getLink(s_anchor.Index - 2).m_cachedWorldTransform *
         m_vecLinks[s_anchor.Index - 1].CenterOfMassOffset;
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
      const btTransform& cLinkTransform =
         GetMultiBody().getBaseWorldTransform() *
         m_vecLinks[s_anchor.Index - 1].CenterOfMassOffset;

      std::cerr << "GetMultiBody().getBaseWorldTransform().getOrigin() = " << GetMultiBody().getBaseWorldTransform().getOrigin() << std::endl;

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

      std::cerr << "UpdateBaseAnchor: " << s_anchor.Id << ", Position = " << s_anchor.Position << ", Orientation = " << s_anchor.Orientation << std::endl;
   }
   
   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::SLink& CDynamics3DPrototypeModel::AddLink(const CLinkEntity& c_link) {
      btVector3 cHalfExtents(c_link.GetExtents().GetX() * 0.5f,
                             c_link.GetExtents().GetZ() * 0.5f,
                             c_link.GetExtents().GetY() * 0.5f);
      /* request collision shape */
      btCollisionShape* pcShape = NULL;
      switch(c_link.GetGeometry()) {
      case CLinkEntity::EGeometry::BOX:
         pcShape = CDynamics3DShapeManager::RequestBox(cHalfExtents);
         break;
      case CLinkEntity::EGeometry::CYLINDER:
         pcShape = CDynamics3DShapeManager::RequestCylinder(cHalfExtents);
         break;
      case CLinkEntity::EGeometry::SPHERE:
         pcShape = CDynamics3DShapeManager::RequestSphere(cHalfExtents.getX());
         break;          
      default:
         THROW_ARGOSEXCEPTION("Geometry not implemented");
         break;
      }
      /* calculate inertia */
      btScalar fMass = c_link.GetMass();
      btVector3 cInertia;
      pcShape->calculateLocalInertia(fMass, cInertia);
      /* calculate the center of mass offset */
      const btTransform& cCenterOfMassOffset = btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -c_link.GetExtents().GetZ() * 0.5f, 0.0f));
      /* calculate transform */
      const CVector3& cPosition = c_link.GetAnchor().Position;
      const CQuaternion& cOrientation = c_link.GetAnchor().Orientation;

      std::cerr << "Adding link: " << c_link.GetId() << ", cPosition = " << cPosition << ", cOrientation = " << cOrientation << std::endl;

      const btTransform& cTransform =
         btTransform(btQuaternion(cOrientation.GetX(),
                                  cOrientation.GetZ(),
                                 -cOrientation.GetY(),
                                  cOrientation.GetW()),
                     btVector3(cPosition.GetX(),
                               cPosition.GetZ(),
                              -cPosition.GetY()));
      /* add to collection */
      m_vecLinks.emplace_back(pcShape, fMass, c_link.GetAnchor(), cInertia, cTransform, cCenterOfMassOffset);
      /* return the link data */
      return m_vecLinks.back();
   }

   /****************************************/
   /****************************************/

   
   CDynamics3DPrototypeModel::CDynamics3DPrototypeModel(CDynamics3DEngine& c_engine,
                                                        CPrototypeEntity& c_entity) :
      CDynamics3DMultiBodyObjectModel(c_engine, c_entity),
      m_cPrototypeEntity(c_entity),
      m_cLinkEquippedEntity(c_entity.GetLinkEquippedEntity()),
      m_cJointEquippedEntity(c_entity.GetJointEquippedEntity()) {
      
      /* use the reference link as the base of the robot */
      CLinkEntity& cBase = c_entity.GetReferenceLink();
      SLink& sBaseLink = AddLink(cBase);
      
      SetBase(m_cLinkEquippedEntity.GetLinks().size() - 1, sBaseLink.Mass, sBaseLink.Inertia);

      RegisterAnchorMethod(cBase.GetAnchor(),
                           &CDynamics3DPrototypeModel::UpdateBaseAnchor);
      cBase.GetAnchor().Enable();

      // iteratively build model using links
      // error conditions: detached link, loop (e.g. attempting to use the parent or an existing link as the child)
      // loop over links until the number of remaining links is zero (done) or the remaining links doesn't change
      // (link detached from model)
      // in base class AddLink(parent, joint type, frame info), SetBaseLink
      for(CJointEntity* pc_joint : m_cJointEquippedEntity.GetAllJoints()) {
         const CLinkEntity& cParentLink = pc_joint->GetParentLink();
         const CLinkEntity& cChildLink = pc_joint->GetChildLink();

         // add check to see if joint/link is already created
         // add check to determine whether the parent exists
         
         std::cerr << pc_joint->GetId() << ".parent_link = "
                   << cParentLink.GetId() << ", anchor = "
                   << cParentLink.GetAnchor().Index << std::endl;
         std::cerr << pc_joint->GetId() << ".child_link = "
                   << cChildLink.GetId() << ", anchor = "
                   << cChildLink.GetAnchor().Index << std::endl;

         if(true) { //parent link exists
            const SLink& sChildLink = AddLink(cChildLink);
            const SLink& sParentLink = m_vecLinks[cParentLink.GetAnchor().Index - 1];

            // setup joint

            /*
            btQuaternion cParentLinkOrientation(cParentLink.GetAnchor().OffsetOrientation.GetX(),
                                                cParentLink.GetAnchor().OffsetOrientation.GetZ(), 
                                                -cParentLink.GetAnchor().OffsetOrientation.GetY(),
                                                cParentLink.GetAnchor().OffsetOrientation.GetW());
            btQuaternion cChildLinkOrientation(cChildLink.GetAnchor().OffsetOrientation.GetX(),
                                               cChildLink.GetAnchor().OffsetOrientation.GetZ(), 
                                               -cChildLink.GetAnchor().OffsetOrientation.GetY(),
                                               cChildLink.GetAnchor().OffsetOrientation.GetW());           
            
            CVector3 cParentToChildOffset = cChildLink.GetAnchor().OffsetPosition - cParentLink.GetAnchor().OffsetPosition;
            */

            // CALCULATE THE TRANSLATION AND ROTATION FROM THE PARENT COM TO THE CHILD COM (AKA THE JOINT FRAME)

            const CVector3& cParentOffsetPosition = pc_joint->GetParentLinkJointPosition();
            const CQuaternion& cParentOffsetOrientation = pc_joint->GetParentLinkJointOrientation();
            std::cerr << "cParentOffsetPosition = " << cParentOffsetPosition << std::endl;
            std::cerr << "cParentOffsetOrientation = " << cParentOffsetOrientation << std::endl;

            btTransform cParentOffsetTransform =
               btTransform(btQuaternion(cParentOffsetOrientation.GetX(),
                                        cParentOffsetOrientation.GetZ(), 
                                       -cParentOffsetOrientation.GetY(),
                                        cParentOffsetOrientation.GetW()),
                           btVector3(cParentOffsetPosition.GetX(),
                                     cParentOffsetPosition.GetZ(),
                                    -cParentOffsetPosition.GetY()));

            std::cerr << "pr.b: " << cParentOffsetTransform;
            cParentOffsetTransform *= (sParentLink.CenterOfMassOffset);
            std::cerr << "pr.a: " << cParentOffsetTransform;

            const CVector3& cChildOffsetPosition = pc_joint->GetChildLinkJointPosition();
            const CQuaternion& cChildOffsetOrientation = pc_joint->GetChildLinkJointOrientation();
            std::cerr << "cChildOffsetPosition = " << cChildOffsetPosition << std::endl;
            std::cerr << "cChildOffsetOrientation = " << cChildOffsetOrientation << std::endl;

            btTransform cChildOffsetTransform =
               btTransform(btQuaternion(cChildOffsetOrientation.GetX(),
                                        cChildOffsetOrientation.GetZ(), 
                                       -cChildOffsetOrientation.GetY(),
                                        cChildOffsetOrientation.GetW()),
                           btVector3(cChildOffsetPosition.GetX(),
                                     cChildOffsetPosition.GetZ(),
                                    -cChildOffsetPosition.GetY()));

            std::cerr << "ch.b: " << cChildOffsetTransform;
            std::cerr << "ch.como: " << sChildLink.CenterOfMassOffset;
            cChildOffsetTransform = cChildOffsetTransform * (sChildLink.CenterOfMassOffset);
            std::cerr << "ch.a: " << cChildOffsetTransform;

            btQuaternion parentRotToThis = cChildOffsetTransform.getRotation() * cParentOffsetTransform.inverse().getRotation();

            switch(pc_joint->GetType()) {
            case CJointEntity::EType::FIXED:
               GetMultiBody().setupFixed(cChildLink.GetAnchor().Index - 2,
                                         sChildLink.Mass,
                                         sChildLink.Inertia,
                                         cParentLink.GetAnchor().Index - 2,
                                         parentRotToThis,
                                         cParentOffsetTransform.getOrigin(),
                                         -cChildOffsetTransform.getOrigin());
               break;
            case CJointEntity::EType::SPHERICAL:
               GetMultiBody().setupSpherical(cChildLink.GetAnchor().Index - 2,
                                             sChildLink.Mass,
                                             sChildLink.Inertia,
                                             cParentLink.GetAnchor().Index - 2,
                                             parentRotToThis,
                                             cParentOffsetTransform.getOrigin(),
                                             -cChildOffsetTransform.getOrigin());
               break;
            default:
               THROW_ARGOSEXCEPTION("Joint type not implemented");
               break;
            }

            RegisterAnchorMethod(cChildLink.GetAnchor(),
                                 &CDynamics3DPrototypeModel::UpdateLinkAnchor);
         }    
      }
      
      
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

      /*
        - parentComToThisPivotOffset is the pivot (translation) from parent body center of mass (COM) to the constraint frame.
        - thisPivotToThisComOffset is the pivot from child center of mass to the constraint frame.
      */

      // TODO: Note that I set the link index here
      // TODO: possible segfault from here, we need to assume that the base or ref link is the anchor with index zero and subtract 1 from all indices since the base link is not included in this array
      // TODO: this seems correct, since if our anchor index is 0, then our base link is -1 which is consistent with the representation in bullet

      // setup
      Finalize();

      // Colliders TODO: loop over an array in the base class

      /* setup collider */
      for(SLink& s_link : m_vecLinks) {
         s_link.Init(this, s_link.Anchor.Index - 2);
         c_engine.GetPhysicsWorld()->addCollisionObject(s_link.Collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
      }
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

