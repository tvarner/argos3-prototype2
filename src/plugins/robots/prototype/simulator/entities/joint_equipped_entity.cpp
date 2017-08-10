/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "joint_equipped_entity.h"

namespace argos {

   SJoint::SJoint(const std::string str_joint_id,
               const EType e_type,
               const CLinkEquippedEntity::SLink* psParentLink,
               const CLinkEquippedEntity::SLink* psChildLink,
               CVector3 c_axis,
               CVector3 c_linear_velocity,
               CVector3 c_angular_velocity,
               bool b_is_enabled) :
      m_strId(str_link_id),
      m_eType(e_geometry),
      m_psParentLink(psParentLink),
      m_psChildLink(psChildLink),
      m_cAxis(c_axis),
      m_cLinearVelocity(c_linear_velocity),
      m_cAngularVelocity(c_angular_velocity),
      m_bIsEnabled(b_is_enabled) {
   }


   /****************************************/
   /****************************************/

   CJointEquippedEntity::CJointEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CJointEquippedEntity::CJointEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CJointEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
        /* Init parent */ // <-- the second constructor is called so no need to call init on composable entity
        // CComposableEntity::Init(t_tree);

        // store initial configuration of joint equipped entity
          /* NOTE: this configuration is valid until prototype entity is 
                   reinitialized
          */
        if (m_ptInitialConfiguration != NULL) { 
          m_ptInitialConfiguration = t_tree->Clone();
        }

        /* Go through the joints */
        TConfigurationNodeIterator itJoint("joint");
        for(itJoint = itJoint.begin(&t_tree);
           itJoint != itJoint.end();
           ++itJoint) {

          // get joint id
          std::string strJointId;
          GetNodeAttribute(*itJoint, 'id', strJointId);

          // get joint axis
          CVector3 cJointAxis;
          GetNodeAttribute(*itJoint, 'axis', cJointAxis);

          // get joint type
          std::string strJointType;
          EType eJointType;
          GetNodeAttribute(t_tree, "type", strJointType);
          if(strJointType == "fixed") {
            eJointType = FIXED;
          }
          else if(strJointType == "prismatic") {
            eJointType = PRISMATIC;
          }
          else if(strJointType == "revolute") {
            eJointType = REVOLUTE;
          }
          else if(strJointType == "spherical") {
            eJointType = SPHERICAL;
          }
          else if(strJointType == "planar") {
            eJointType = PLANAR;
          }
          else {
            THROW_ARGOSEXCEPTION("Joint type \"" << strJointType << "\" is not implemented");
          }

          // get link equipped entity
          CLinkEquippedEntity* cLinkEquippedEntity = GetParent().GetComponent<CLinkEquippedEntity>("links");

          // get parent node
          TConfigurationNode& tJointParentNode = GetNode(t_tree, "parent");
          
          // get parent link id attr from parent node
          std::string strJointParentLinkId;
          GetNodeAttribute(tJointParentNode, "link", strJointParentLinkId);

          // use parent link id to get parent SLink from link equipped entity
          psParentLink = &(cLinkEquippedEntity.GetLink(strJointParentLinkId));
          GetNodeAttributeOrDefault(tJointParentNode, "position", m_cParentLinkJointPosition, m_cParentLinkJointPosition);
          GetNodeAttributeOrDefault(tJointParentNode, "orientation", m_cParentLinkJointOrientation, m_cParentLinkJointOrientation);

          // set parent position on SLink from parent node
          GetNodeAttributeOrDefault(tJointParentNode, 
                                "position", 
                                psParentLink->m_psAnchor->Position, 
                                psParentLink->m_psAnchor->Position);

          // set parent orientation on parent SLink from parent node
          CVector3 cParentLinkOrientation;
          GetNodeAttributeOrDefault(tJointParentNode, 
                                "orientation", 
                                psParentLink->m_psAnchor->Orientation, 
                                psParentLink->m_psAnchor->Orientation);

          // get child node
          TConfigurationNode& tJointChildNode = GetNode(t_tree, "parent");

          // get child link id attr from child node 
          std::string strJointChildLinkId;
          GetNodeAttribute(tJointChildNode, "link", strJointChildLinkId);

          // use child link id to get child SLink from link equipped entity
          psChildLink = &(cLinkEquippedEntity.GetLink(strJointChildLinkId));
          GetNodeAttributeOrDefault(tJointChildNode, 
                                "position", 
                                psChildLink->m_psAnchor->Orientation, 
                                psChildLink->m_psAnchor->Orientation);

          Real fInitialLinearVelocity = 0.0f;
          Real fInitialAngularVelocity = 0.0f;

          // create new joint
          SJoint* joint = new SJoint(strJointId,
                                    eJointType,
                                    psParentLink,
                                    psChildLink,
                                    cJointAxis,
                                    fInitialLinearVelocity,
                                    fInitialAngularVelocity)

          // add joint to joints map
          m_tJoints.push_back(joint);
       }
    }
    catch(CARGoSException& ex) {
       THROW_ARGOSEXCEPTION_NESTED("Failed to initialize joint equipped entity \"" << GetId() << "\".", ex);
    }
  }

   /****************************************/
   /****************************************/

    void CJointEquippedEntity::Reset() {
      Init(*m_ptInitialConfiguration);
    }

   /****************************************/
   /****************************************/

   SJoint& CJointEquippedEntity::GetJoint(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tJoints.size(),
                   "CJointEquippedEntity::GetJoint(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tJoints.size() = " <<
                   m_tJoints.size());
      return *m_tJoints[un_index];
   }

   /****************************************/
   /****************************************/

   void CJointEquippedEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CJointEquippedEntity);

   /****************************************/
   /****************************************/

}
