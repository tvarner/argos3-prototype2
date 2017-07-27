/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/link_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef LINK_ENTITY_H
#define LINK_ENTITY_H

namespace argos {
   class CLinkEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <unordered_map>

namespace argos {

   // the advantage of using a positional entity is that we cache the location for a visualization plugin
   // however, if this is the only use, we should remove this and just use the anchor to calculate

   // TODO: collapse this entity into a simple struct inside link equipped entity
   class CLinkEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      using TMap = std::unordered_map<std::string, CLinkEntity*>;

   public:
      enum EGeometry {
         CYLINDER,
         BOX,
         SPHERE,
      };

   public:

      CLinkEntity(CComposableEntity* pc_parent);

      virtual ~CLinkEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Destroy();

      virtual std::string GetTypeDescription() const {
         return "link";
      }

      EGeometry GetGeometry() const {
         return m_eGeometry;
      }

      const CVector3& GetExtents() const {
         return m_cExtents;
      }

      Real GetMass() const {
         return m_fMass;
      }

      // To decide, perhaps it is more appropiate to have a struct inside link equipped entity than this class and to store a reference to the anchor
      SAnchor& GetAnchor() {
         return *m_psAnchor;
      } 

      const SAnchor& GetAnchor() const {
         return *m_psAnchor;
      } 


   private:

		EGeometry m_eGeometry;
		CVector3 m_cExtents;
		Real m_fMass;
      SAnchor* m_psAnchor;
   };

}

#endif
