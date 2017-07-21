/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef LINK_EQUIPPED_ENTITY_H
#define LINK_EQUIPPED_ENTITY_H

namespace argos {
   class CLinkEquippedEntity;
   class CLinkEntity;
}

#include <argos3/plugins/robots/prototype/simulator/entities/link_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <map>

namespace argos {

   class CLinkEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CLinkEquippedEntity(CComposableEntity* pc_parent);

      CLinkEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Update() {}

      //CLinkEntity& GetLink(UInt32 un_index);

      CLinkEntity& GetLink(const std::string& str_link_id) {
         return *m_tLinks[str_link_id];
      }

      CLinkEntity& GetBase() {
         return *m_pcBase;
      }

      inline CLinkEntity::TMap& GetAllLinks() {
         return m_tLinks;
      }

      inline size_t GetNumLinks() {
         return m_tLinks.size();
      }


      virtual std::string GetTypeDescription() const {
         return "links";
      }

   protected:

      virtual void UpdateComponents();

   protected:
      // todo, why is this protected? use link 0 as base link
      CLinkEntity::TMap m_tLinks;

      CLinkEntity* m_pcBase;
   };

}

#endif
