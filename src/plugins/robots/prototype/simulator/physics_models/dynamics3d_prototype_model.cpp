/**
 * @file <argos3/plugins/robots/prototype/simulator/physics_models/dynamics3d_prototype_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_prototype_model.h"

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

#include <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/frame_equipped_entity.h>

#include <argos3/plugins/robots/prototype/utility/geometry3.h>

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::CDynamics3DPrototypeModel(CDynamics3DEngine& c_engine,
                                                CPrototypeEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity(), c_entity.GetId()),
      m_cPrototypeEntity(c_entity),
      m_cLinkEquippedEntity(c_entity.GetLinkEquippedEntity()),
      m_cJointEquippedEntity(c_entity.GetJointEquippedEntity()) {

      /* reserve memory */
      size_t unNumLinks = m_cLinkEquippedEntity.GetNumLinks();
      m_vecLinkInertias.resize(m_cLinkEquippedEntity.GetNumLinks());
      m_vecLinkShapes.resize(m_cLinkEquippedEntity.GetNumLinks());
      
      /* setup the base */
      CLinkEntity& cBase = m_cLinkEquippedEntity.GetBase();

      btVector3 cBaseHalfExtents(cBase.GetExtents().GetX() * 0.5f,
                                 cBase.GetExtents().GetZ() * 0.5f,
                                 cBase.GetExtents().GetY() * 0.5f);
      
      /* TODO: factorize the link setup for the base and other links into base class */
      switch(cBase.GetGeometry()) {
         case CLinkEntity::EGeometry::BOX:
            pcBaseShape = CDynamics3DShapeManager::RequestBox(cBaseHalfExtents);
            break;
         default:
            THROW_ARGOSEXCEPTION("Not implemented");
            break;
      }

      // TODO: implement a check to determine if inertia is already defined (e.g. via URDF)
      btScalar fBaseMass = m_cBase.GetMass();
      
      pcBaseShape->calculateLocalInertia(fBaseMass, m_vecLinkInertias[0]);
     
      SetBody(m_cLinkEquippedEntity.GetNumLinks(), fBaseMass, m_vecLinkInertias[0]);

      // iteratively build model using links

      // error conditions: detached link, loop (e.g. attempting to use the parent or an existing link as the child)
      // loop over links until the number of remaining links is zero (done) or the remaining links doesn't change (link detached from model)
      // in base class AddLink(parent, joint type, frame info), SetBaseLink
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
         
         switch(cChildLink.GetGeometry()) {
         case CLinkEntity::EGeometry::BOX:
            pcShape = CDynamics3DShapeManager::RequestBox(cHalfExtents);
            break;
         default:
            THROW_ARGOSEXCEPTION("Link geometry not implemented");
            break;
         }

         btVector3 cInertia; // to store in vector, also to be stored is the collision shape for adding the object at the end of this loop
         pcShape->calculateLocalInertia(fBaseMass, cInertia);

         btQuaternion cParentToChild()
         
         switch((*itJoint)->GetType()) {
         case CJointEntity::EType::FIXED:
            

            // WARNING: HARDED CODED TEST, 0->1
            GetMultiBody().setupFixed(1, cChildLink.GetMass(), cInertia, 0,  )
            break;
         default:
            THROW_ARGOSEXCEPTION("Joint type not implemented");
            break;
         }


         
      }






      	pMultiBody->finalizeMultiDof();

	pWorld->addMultiBody(pMultiBody);

      
      for(CLinkEntity::TList::iterator itLink = m_cLinkEquippedEntity.GetAllLinks().begin();
          itLink != m_cLinkEquippedEntity.GetAllLinks().end();
          ++itLink) {

         btCollisionShape* pcShape = NULL;

         /* check the tag to determine which shape manager to use */
         switch((*itLink)->GetGeometry()) {
         case CLinkEntity::EGeometry::BOX:
            pcShape = CDynamics3DShapeManager::RequestBox(cHalfExtents);
            break;
         case CLinkEntity::EGeometry::CYLINDER:
            THROW_ARGOSEXCEPTION("Not implemented");
            break;
         case CLinkEntity::EGeometry::SPHERE:
            THROW_ARGOSEXCEPTION("Not implemented");
            break;
         }
         btTransform cPositionalOffset(ARGoSToBullet((*itBody)->GetOffsetPositionalEntity().GetOrientation()),
                                       ARGoSToBullet((*itBody)->GetOffsetPositionalEntity().GetPosition()));
         btTransform cGeometricOffset(btQuaternion(0,0,0,1), btVector3(0, -cHalfExtents.getY(), 0));
         
         m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                        (*itBody)->GetId(), 
                                                        pcShape, 
                                                        cPositionalOffset,
                                                        cGeometricOffset,
                                                        (*itBody)->GetMass()));
      }

      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {
         CFrameEntity::TList& tFrames = (*itJoint)->GetFrameEquippedEntity().GetAllFrames();
         if(tFrames.size() != 2) {
            THROW_ARGOSEXCEPTION("This version of the Dynamics3D plugin only allows joints between two bodies");
         }
         CDynamics3DBody::TVector::iterator itDyn3dBody0 = std::find(m_vecLocalBodies.begin(),
                                                                     m_vecLocalBodies.end(),
                                                                     tFrames[0]->GetBodyEntity().GetId());
         if(itDyn3dBody0 == m_vecLocalBodies.end()) {
            THROW_ARGOSEXCEPTION("Unable to find referenced body \"" <<
                                 tFrames[0]->GetBodyEntity().GetId() << "\"." );
         }
         CDynamics3DBody::TVector::iterator itDyn3dBody1 = std::find(m_vecLocalBodies.begin(),
                                                                     m_vecLocalBodies.end(),
                                                                     tFrames[1]->GetBodyEntity().GetId());
         if(itDyn3dBody1 == m_vecLocalBodies.end()) {
            THROW_ARGOSEXCEPTION("Unable to find referenced body \"" <<
                                 tFrames[1]->GetBodyEntity().GetId() << "\"." );
         }

         /* Get the frames in each respective bodies */
         btTransform cFrameOriginInBody0 = (*itDyn3dBody0)->GetGeometricOffset() *
            btTransform(ARGoSToBullet(tFrames[0]->GetPositionalEntity().GetOrientation()),
                        ARGoSToBullet(tFrames[0]->GetPositionalEntity().GetPosition()));         
         btTransform cFrameOriginInBody1 = (*itDyn3dBody1)->GetGeometricOffset() *
            btTransform(ARGoSToBullet(tFrames[1]->GetPositionalEntity().GetOrientation()),
                        ARGoSToBullet(tFrames[1]->GetPositionalEntity().GetPosition()));
         

         /* Get the limits of the joint - we must manually swap Z and Y here! */
         /* linear */
         btVector3 cLinearLowerLimit((*itJoint)->GetAxisLinearX().m_bUnconstrained ?
                                     1.0f : (*itJoint)->GetAxisLinearX().m_cLimits.GetMin(),
                                     (*itJoint)->GetAxisLinearZ().m_bUnconstrained ?
                                     1.0f : (*itJoint)->GetAxisLinearZ().m_cLimits.GetMin(),
                                     (*itJoint)->GetAxisLinearY().m_bUnconstrained ?
                                     1.0f : (*itJoint)->GetAxisLinearY().m_cLimits.GetMin());
         btVector3 cLinearUpperLimit((*itJoint)->GetAxisLinearX().m_bUnconstrained ?
                                     -1.0f : (*itJoint)->GetAxisLinearX().m_cLimits.GetMax(),
                                     (*itJoint)->GetAxisLinearZ().m_bUnconstrained ?
                                     -1.0f : (*itJoint)->GetAxisLinearZ().m_cLimits.GetMax(),
                                     (*itJoint)->GetAxisLinearY().m_bUnconstrained ?
                                     -1.0f : (*itJoint)->GetAxisLinearY().m_cLimits.GetMax());               
         /* angular */
         btVector3 cAngularLowerLimit((*itJoint)->GetAxisAngularX().m_bUnconstrained ?
                                      1.0f : (*itJoint)->GetAxisAngularX().m_cLimits.GetMin().GetValue(),
                                      (*itJoint)->GetAxisAngularZ().m_bUnconstrained ?
                                      1.0f : (*itJoint)->GetAxisAngularZ().m_cLimits.GetMin().GetValue(),
                                      (*itJoint)->GetAxisAngularY().m_bUnconstrained ?
                                      1.0f : (*itJoint)->GetAxisAngularY().m_cLimits.GetMin().GetValue());
         btVector3 cAngularUpperLimit((*itJoint)->GetAxisAngularX().m_bUnconstrained ?
                                      -1.0f : (*itJoint)->GetAxisAngularX().m_cLimits.GetMax().GetValue(),
                                      (*itJoint)->GetAxisAngularZ().m_bUnconstrained ?
                                      -1.0f : (*itJoint)->GetAxisAngularZ().m_cLimits.GetMax().GetValue(),
                                      (*itJoint)->GetAxisAngularY().m_bUnconstrained ?
                                      -1.0f : (*itJoint)->GetAxisAngularY().m_cLimits.GetMax().GetValue());

         CDynamics3DJoint::SJointActuators sLinearActuators(
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisLinearX().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisLinearX().m_fActuatorForce,
                                                         (*itJoint)->GetAxisLinearX().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisLinearZ().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisLinearZ().m_fActuatorForce,
                                                         (*itJoint)->GetAxisLinearZ().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisLinearY().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisLinearY().m_fActuatorForce,
                                                         (*itJoint)->GetAxisLinearY().m_fActuatorTargetVelocity));       
         CDynamics3DJoint::SJointActuators sAngularActuators(
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisAngularX().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisAngularX().m_fActuatorForce,
                                                         (*itJoint)->GetAxisAngularX().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisAngularZ().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisAngularZ().m_fActuatorForce,
                                                         (*itJoint)->GetAxisAngularZ().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisAngularY().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisAngularY().m_fActuatorForce,
                                                         (*itJoint)->GetAxisAngularY().m_fActuatorTargetVelocity));
         
         /* create the joint */
         m_vecLocalJoints.push_back(new CDynamics3DJoint((*itJoint)->GetId(),
                                                         **itDyn3dBody0,
                                                         **itDyn3dBody1,
                                                         cFrameOriginInBody0,
                                                         cFrameOriginInBody1,
                                                         CDynamics3DJoint::SJointLimits(cLinearLowerLimit, cLinearUpperLimit),
                                                         CDynamics3DJoint::SJointLimits(cAngularLowerLimit, cAngularUpperLimit),
                                                         sLinearActuators,
                                                         sAngularActuators,
                                                         true,
                                                         (*itJoint)->GetDisableLinkedBodyCollisions()));
      }
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                      ARGoSToBullet(GetEmbodiedEntity().GetPosition())));
      /* Update the bodies inside the entity which have their positions driven by the physics engines */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::~CDynamics3DPrototypeModel() {

      //@todo clean up collision shapes?

      /*
      CDynamics3DBody::TVector::iterator itBody;
      std::vector<SConstraint>::iterator itConstraint;

      for(itConstraint = m_vecLocalConstraints.begin();
          itConstraint != m_vecLocalConstraints.end();
          ++itConstraint) {
         delete itConstraint->m_pcConstraint;
      }
      
      for(itBody = m_vecLocalBodies.begin();
          itBody != m_vecLocalBodies.end();
          ++itBody) {
         delete itBody->second->m_pcCollisionShape;
      }
      */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DPrototypeModel::UpdateEntityStatus() {
      /* Update prototype position and orientation */
      btTransform cBodyTransform;
      

      /* Update the position of the bodies */
      //  link to positional entities on init?
      for(CBodyEntity::TList::iterator itBody = m_cBodyEquippedEntity.GetAllBodies().begin();
          itBody != m_cBodyEquippedEntity.GetAllBodies().end();
          ++itBody) {
         
         //@todo optimise by storing a pointer
         CDynamics3DBody& cBody = **std::find(m_vecLocalBodies.begin(),
                                              m_vecLocalBodies.end(),
                                              (*itBody)->GetId());
         
         //@todo move this line into the UpdateFromEntityStatus method, it doesn't belong here
         cBody.ActivateRigidBody();

         (*itBody)->GetPositionalEntity().SetPosition(BulletToARGoS(cBody.GetMotionStateTransform().getOrigin()));      
         (*itBody)->GetPositionalEntity().SetOrientation(BulletToARGoS(cBody.GetMotionStateTransform().getRotation()));
      }
      
      /* Write back the joint angles and positions */
      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {
         
         //@todo optimise by storing a pointer
         CDynamics3DJoint& cJoint = **std::find(m_vecLocalJoints.begin(),
                                                m_vecLocalJoints.end(),
                                                (*itJoint)->GetId());
         
         btTransform cJointTransform = cJoint.GetSensorReading();

         (*itJoint)->SetJointRotation(BulletToARGoS(cJointTransform.getRotation()));
         (*itJoint)->SetJointTranslation(BulletToARGoS(cJointTransform.getOrigin()));
      }

      /* Update the emboddied entity */
      const btTransform& cEntityUpdateTransform = GetModelCoordinates();
      
      GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityUpdateTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityUpdateTransform.getRotation()));

      /* Update components */
      m_cPrototypeEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DPrototypeModel::UpdateFromEntityStatus() {
      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {
         
         CDynamics3DJoint& cJoint = **std::find(m_vecLocalJoints.begin(),
                                                m_vecLocalJoints.end(),
                                                (*itJoint)->GetId());
         /* Write the motor target velocities to the joints */
         /* add method for enable/disable? */
         //@todo for correctness it might be necerry to negate the z/y velocities according to the transforms

         cJoint.SetActuatorTargetVelocity(CDynamics3DJoint::LINEAR_X,
                                          (*itJoint)->GetAxisLinearX().m_fActuatorTargetVelocity);
         cJoint.SetActuatorTargetVelocity(CDynamics3DJoint::LINEAR_Y,
                                          -(*itJoint)->GetAxisLinearZ().m_fActuatorTargetVelocity);
         cJoint.SetActuatorTargetVelocity(CDynamics3DJoint::LINEAR_Z,
                                          (*itJoint)->GetAxisLinearY().m_fActuatorTargetVelocity);

         cJoint.SetActuatorTargetVelocity(CDynamics3DJoint::ANGULAR_X,
                                          (*itJoint)->GetAxisAngularX().m_fActuatorTargetVelocity);
         cJoint.SetActuatorTargetVelocity(CDynamics3DJoint::ANGULAR_Y,
                                          (*itJoint)->GetAxisAngularZ().m_fActuatorTargetVelocity);
         cJoint.SetActuatorTargetVelocity(CDynamics3DJoint::ANGULAR_Z,
                                          (*itJoint)->GetAxisAngularY().m_fActuatorTargetVelocity);
      }

      //GetReferenceBody().ActivateRigidBody();
      // Add method get reference body which sets the pointer
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DPrototypeModel::GetModelCoordinates() const {
      const std::string& strReferenceBodyId = 
         m_cPrototypeEntity.GetBodyEquippedEntity().GetReferenceBody().GetId();
      
      //@todo optimise by storing pointers
      const CDynamics3DBody& sReferenceBodyConfiguration = **std::find(m_vecLocalBodies.begin(),
                                                                       m_vecLocalBodies.end(),
                                                                       strReferenceBodyId);

      return (sReferenceBodyConfiguration.GetMotionStateTransform() *
              sReferenceBodyConfiguration.GetPositionalOffset().inverse());
   }

   /****************************************/
   /****************************************/

   btBoxShape* CDynamics3DPrototypeModel::CBoxShapeManager::RequestBoxShape(const btVector3& c_half_extents) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cHalfExtents == c_half_extents) break;
      }      
      // if it doesn't exist, create a new one
      if(itResource == m_vecResources.end()) {
         itResource = m_vecResources.insert(itResource, 
                                            CResource(c_half_extents, new btBoxShape(c_half_extents)));
      }
      itResource->m_unInUseCount++;
      return itResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DPrototypeModel::CBoxShapeManager::ReleaseBoxShape(const btBoxShape* pc_release) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itResource == m_vecResources.end()) {
         THROW_ARGOSEXCEPTION("Attempt to release unknown btBoxShape from the box shape manager!");
      }
      itResource->m_unInUseCount--;
      if(itResource->m_unInUseCount == 0) {
         delete itResource->m_cShape;
         m_vecResources.erase(itResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::CBoxShapeManager::CResource::CResource(const btVector3& c_half_extents,
                                                                 btBoxShape* c_shape) : 
      m_cHalfExtents(c_half_extents),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   btCylinderShape* CDynamics3DPrototypeModel::CCylinderShapeManager::RequestCylinderShape(const btVector3& c_half_extents) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cHalfExtents == c_half_extents) break;
      }      
      // if it doesn't exist, create a new one
      if(itResource == m_vecResources.end()) {
         itResource = m_vecResources.insert(itResource, 
                                            CResource(c_half_extents, new btCylinderShape(c_half_extents)));
      }
      itResource->m_unInUseCount++;
      return itResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DPrototypeModel::CCylinderShapeManager::ReleaseCylinderShape(const btCylinderShape* pc_release) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itResource == m_vecResources.end()) {
         THROW_ARGOSEXCEPTION("Attempt to release unknown btCylinderShape from the cylinder shape manager!");
      }
      itResource->m_unInUseCount--;
      if(itResource->m_unInUseCount == 0) {
         delete itResource->m_cShape;
         m_vecResources.erase(itResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::CCylinderShapeManager::CResource::CResource(const btVector3& c_half_extents,
                                                                      btCylinderShape* c_shape) : 
      m_cHalfExtents(c_half_extents),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   btSphereShape* CDynamics3DPrototypeModel::CSphereShapeManager::RequestSphereShape(Real f_radius) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_fRadius == f_radius) break;
      }      
      // if it doesn't exist, create a new one
      if(itResource == m_vecResources.end()) {
         itResource = m_vecResources.insert(itResource, 
                                            CResource(f_radius, new btSphereShape(f_radius)));
      }
      itResource->m_unInUseCount++;
      return itResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DPrototypeModel::CSphereShapeManager::ReleaseSphereShape(const btSphereShape* pc_release) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itResource == m_vecResources.end()) {
         THROW_ARGOSEXCEPTION("Attempt to release unknown btSphereShape from the sphere shape manager!");
      }
      itResource->m_unInUseCount--;
      if(itResource->m_unInUseCount == 0) {
         delete itResource->m_cShape;
         m_vecResources.erase(itResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::CSphereShapeManager::CResource::CResource(Real f_radius,
                                                                    btSphereShape* c_shape) : 
      m_fRadius(f_radius),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   CDynamics3DPrototypeModel::CBoxShapeManager      CDynamics3DPrototypeModel::m_cBoxShapeManager;
   CDynamics3DPrototypeModel::CCylinderShapeManager CDynamics3DPrototypeModel::m_cCylinderShapeManager;
   CDynamics3DPrototypeModel::CSphereShapeManager   CDynamics3DPrototypeModel::m_cSphereShapeManager;

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CPrototypeEntity, CDynamics3DPrototypeModel);

   /****************************************/
   /****************************************/

}

