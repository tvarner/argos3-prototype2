/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "link_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CLinkEquippedEntity::CLinkEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CLinkEquippedEntity::CLinkEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
          /* the second constructor is called so no need to call init on composable entity */
         // CComposableEntity::Init(t_tree);

         TConfigurationNodeIterator itLink("link");
         for(itLink = itLink.begin(&t_tree);
             itLink != itLink.end();
             ++itLink) {
            
            CLinkEntity* pcLinkEntity = new CLinkEntity(this);
            pcLinkEntity->Init(*itLink);
            AddComponent(*pcLinkEntity);            
            m_tLinks[pcLinkEntity->GetId()] = pcLinkEntity;
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize link equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CLinkEquippedEntity);

   /****************************************/
   /****************************************/

}
