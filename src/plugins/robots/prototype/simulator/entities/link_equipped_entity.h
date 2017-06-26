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

      typedef std::map<std::string, CLinkEquippedEntity*> TMap;

   public:

      CLinkEquippedEntity(CComposableEntity* pc_parent);

      CLinkEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Update() {}
      /*
      void AddLink(const CVector3& c_position,
                   const CQuaternion& c_orientation,
                   const CVector3& c_size,
                   Real f_mass);
      */
      CLinkEntity& GetLink(UInt32 un_index);

      CLinkEntity& GetReferenceLink() {
         return *m_pcReferenceLink;
      }

      inline CLinkEntity::TList& GetAllBodies() {
         return m_tBodies;
      }

      virtual std::string GetTypeDescription() const {
         return "bodies";
      }

   protected:

      virtual void UpdateComponents();

   protected:

      CLinkEntity::TList m_tBodies;

      CLinkEntity * m_pcReferenceLink;
   };

}

#endif
