/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/link_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef LINK_ENTITY_H
#define LINK_ENTITY_H

namespace argos {
   class CLinkEntity;
   class CPositionalEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/positional_entity.h>

namespace argos {

   class CLinkEntity : public CPositionalEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CLinkEntity*> TList;
      typedef std::vector<CLinkEntity*>::iterator TListIterator;
      typedef std::vector<CLinkEntity*>::const_iterator TListConstIterator;

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

   private:

		EGeometry m_eGeometry;
		CVector3 m_cExtents;
		Real m_fMass;
      SAnchor* m_psAnchor;
   };

}

#endif
