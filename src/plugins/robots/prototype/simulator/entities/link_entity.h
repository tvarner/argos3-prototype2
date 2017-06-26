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
#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/plugins/robots/prototype/utility/geometry3.h>

namespace argos {

   class CLinkEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CLinkEntity*> TList;
      typedef std::vector<CLinkEntity*>::iterator TListIterator;
      typedef std::vector<CLinkEntity*>::const_iterator TListConstIterator;

   public:

      CLinkEntity(CComposableEntity* pc_parent);

      CLinkEntity(CComposableEntity* pc_parent,
                  const std::string& str_id,
                  const CGeometry3& c_geometry,
                  const CVector3& c_offset_position,
                  const CQuaternion& c_offset_orientation,
                  Real f_mass);

      virtual ~CLinkEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CPositionalEntity& GetPositionalEntity() {
         return *m_pcPositionalEntity;
      }

      const CPositionalEntity& GetPositionalEntity() const {
         return *m_pcPositionalEntity;
      }
      
      CPositionalEntity& GetOffsetPositionalEntity() {
         return *m_pcOffsetPositionalEntity;
      }
      
      const CPositionalEntity& GetOffsetPositionalEntity() const {
         return *m_pcOffsetPositionalEntity;
      } 

      virtual std::string GetTypeDescription() const {
         return "link";
      }

      const CGeometry3& GetGeometry() const {
         return *m_pcGeometry;
      }

      Real GetMass() const {
         return m_fMass;
      }

   private:

      CPositionalEntity* m_pcPositionalEntity;
      CPositionalEntity* m_pcOffsetPositionalEntity;

      CGeometry3* m_pcGeometry;

public: // hack - make this private again
      Real m_fMass;
   };

}

#endif
