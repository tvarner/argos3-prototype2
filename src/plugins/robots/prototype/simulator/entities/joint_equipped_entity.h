/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef JOINT_EQUIPPED_ENTITY_H
#define JOINT_EQUIPPED_ENTITY_H

namespace argos {
   class CJointEquippedEntity;
   class CJointEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <map>

namespace argos {

   class CJointEquippedEntity : public CComposableEntity {

   public:

      enum EType { 
         FIXED,
         PRISMATIC,
         REVOLUTE,
         SPHERICAL,
         PLANAR
      };

      struct SJoint { 
         SJoint(const std::string str_joint_id,
               const EType e_type,
               const CLinkEquippedEntity::SLink* m_psParentLink,
               const CLinkEquippedEntity::SLink* m_psChildLink,
               CVector3 c_axis,
               CVector3 c_linear_velocity,
               CVector3 c_angular_velocity,
               bool isEnabled);

         std::string m_strId;
         EType m_eType;
         CLinkEquippedEntity::SLink* m_psParentLink;
         CLinkEquippedEntity::SLink* m_psChildLink;
         CVector3 m_cAxis;
         CVector3 m_cLinearVelocity;
         CVector3 m_cAngularVelocity;
         bool m_bIsEnabled;
      };

   public:

      ENABLE_VTABLE();

      typedef std::vector<SJoint*> TJoints;

   public:

      CJointEquippedEntity(CComposableEntity* pc_parent);

      CJointEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update() {}

      SJoint& GetJoint(UInt32 un_index);

      inline TJoints& GetAllJoints() {
         return m_tJoints;
      }

      virtual std::string GetTypeDescription() const {
         return "joints";
      }

      inline void SetActuatorParametersJointVelocity(std::string str_joint_id,
                                                      CVector3 e_joint_axis,
                                                      Real f_target_velocity);

      virtual void UpdateComponents();

   protected:

      TJoints m_tJoints;

      TConfigurationNode m_tInitialConfiguration;
   };

}

#endif
