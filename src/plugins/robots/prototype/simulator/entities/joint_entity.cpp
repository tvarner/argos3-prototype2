/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/joint_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "joint_entity.h"

#include <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CJointEntity::CJointEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent),
      m_bDisableCollisions(false) {}

   /****************************************/
   /****************************************/

   
   CJointEntity::CJointEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              bool b_disable_collisions) :
      CComposableEntity(pc_parent, str_id),
      m_bDisableCollisions(b_disable_collisions) {
   }

   /****************************************/
   /****************************************/

   /*
      <joint id="box1:box2" type="fixed">
         <parent link="box1" position="0,0,.1" orientation="0,0,0"/>
         <child link="box2" position="0,0,0" orientation="0,0,0"/>
      </joint>

      CVector3 m_cParentLinkJointOffsetPosition;
      CVector3 m_cChildLinkJointOffsetPosition; 
      CQuaternion m_cParentLinkJointOffsetOrientation;
      CQuaternion m_cChildLinkJointOffsetOrientation;
   */

   void CJointEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Get parent and child links */
         // TODO: consider collapsing joint into a simple struct.
         CLinkEquippedEntity& cLinkEquippedEntity = 
            GetParent().GetParent().GetComponent<CLinkEquippedEntity>("links");
         /* Get joint type */
         std::string strJointType;
         GetNodeAttribute(t_tree, "type", strJointType);
         if(strJointType == "fixed") {
            m_eType = FIXED;
         }
         else if(strJointType == "prismatic") {
            m_eType = PRISMATIC;
         }
         else if(strJointType == "revolute") {
            m_eType = REVOLUTE;
         }
         else if(strJointType == "spherical") {
            m_eType = SPHERICAL;
         }
         else if(strJointType == "planar") {
            m_eType = PLANAR;
         }
         else {
            THROW_ARGOSEXCEPTION("Joint type \"" << strJointType << "\" is not implemented");
         }
         /* check if we are disabling collisions */
         GetNodeAttributeOrDefault(t_tree, "disable_collisions", m_bDisableCollisions, false);
         /* parse the parent node */
         TConfigurationNode& tJointParentNode = GetNode(t_tree, "parent");
         std::string strJointParentLink;
         GetNodeAttribute(tJointParentNode, "link", strJointParentLink);
         m_pcParentLink = &(cLinkEquippedEntity.GetLink(strJointParentLink));
         GetNodeAttributeOrDefault(tJointParentNode, "position", m_cParentLinkJointPosition, m_cParentLinkJointPosition);
         GetNodeAttributeOrDefault(tJointParentNode, "orientation", m_cParentLinkJointOrientation, m_cParentLinkJointOrientation);
         /* parse the child node */
         TConfigurationNode& tJointChildNode = GetNode(t_tree, "child");
         std::string strJointChildLink;
         GetNodeAttribute(tJointChildNode, "link", strJointChildLink);
         m_pcChildLink = &(cLinkEquippedEntity.GetLink(strJointChildLink));
         GetNodeAttributeOrDefault(tJointChildNode, "position", m_cChildLinkJointPosition, m_cChildLinkJointPosition);
         GetNodeAttributeOrDefault(tJointChildNode, "orientation", m_cChildLinkJointOrientation, m_cChildLinkJointOrientation);

         
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing joint entity", ex);
      }
   }

   /****************************************/
   /****************************************/


   void CJointEntity::Reset() {
      CComposableEntity::Reset();
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CJointEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CJointEntity);

   /****************************************/
   /****************************************/

}
