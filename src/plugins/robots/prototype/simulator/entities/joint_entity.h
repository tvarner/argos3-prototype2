/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/joint_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef JOINT_ENTITY_H
#define JOINT_ENTITY_H

namespace argos {
   class CLinkEntity;
   class CJointEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CJointEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CJointEntity*> TList;

   public:
      enum EType {
         FIXED,
         PRISMATIC,
         REVOLUTE,
         SPHERICAL,
         PLANAR,
      };

      EType GetType() const {
         return m_eType;
      }

   public:

      CJointEntity(CComposableEntity* pc_parent);

      CJointEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   bool b_disable_collisions);

      virtual ~CJointEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      virtual std::string GetTypeDescription() const {
         return "joint";
      }

      CLinkEntity& GetParentLink();
      
      CLinkEntity& GetChildLink();

      bool GetDisableLinkedBodyCollisions() const {
         return m_bDisableCollisions;
      }

   private:
      bool m_bDisableCollisions;

      EType m_eType;

      CLinkEntity* m_pcParentLink;
      CLinkEntity* m_pcChildLink;
   };

}

#endif
