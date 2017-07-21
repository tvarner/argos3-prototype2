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

   void CJointEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Get parent and child links */
         // TODO: consider collapsing joint into a simple struct.
         CLinkEquippedEntity& cLinkEquippedEntity = GetParent().GetParent().GetComponent<CLinkEquippedEntity>("links");
         std::string strParentLink;
         GetNodeAttribute(t_tree, "parent", strParentLink);
         m_pcParentLink = &(cLinkEquippedEntity.GetLink(strParentLink));
         std::string strChildLink;
         GetNodeAttribute(t_tree, "child", strChildLink);
         m_pcChildLink = &(cLinkEquippedEntity.GetLink(strChildLink));
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
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing joint entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   CLinkEntity& CJointEntity::GetParentLink() {
      return *m_pcParentLink;
   }

   /****************************************/
   /****************************************/
      
   CLinkEntity& CJointEntity::GetChildLink() {
      return *m_pcChildLink;
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
