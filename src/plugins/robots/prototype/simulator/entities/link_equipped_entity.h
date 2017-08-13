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

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <map>

namespace argos {

   class CLinkEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      enum EGeometry { 
         CYLINDER,
         BOX,
         SPHERE
      };

      class SLink { 
         SLink(const std::string str_link_id,
               const EGeometry e_geometry,
               const Real f_mass,
               const CVector3 c_extents,
               const SAnchor& ps_anchor);

         std::string m_strId;
         Real m_fMass;
         EGeometry m_eGeometry;
         CVector3 m_cExtents;
         SAnchor* m_psAnchor;

         Reset();
      };

   public:

      typedef std::map<std::string, SLink*> TLinks;

   public:

      CLinkEquippedEntity(CComposableEntity* pc_parent);

      CLinkEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();
      
      virtual void Update() {}
 
      SLink& GetLink(const std::string& str_link_id) {
         return *m_tLinks[str_link_id];
      }

      inline TLinks& GetAllLinks() {
         return *m_vecLinks;
      }

      SLink& GetBase() {
         return *m_pcBase;
      }

      virtual std::string GetTypeDescription() const {
         return "links";
      }

   protected:
      // todo, why is this protected? use link 0 as base link
      TLinks m_tLinks;

      TConfigurationNode = m_cInitialConfiguration;

      SLink m_pcBase;
   };

}

#endif
