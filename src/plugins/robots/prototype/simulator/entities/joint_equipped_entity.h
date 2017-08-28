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

      // make this and CLink a class, all members private, add
      // getters and setters (set actuator params linear, angular xyz)
      class CJoint { 
         CJoint(const std::string str_joint_id,
               const EType e_type,
               const CLinkEquippedEntity::CLink* m_psParentLink,
               const CLinkEquippedEntity::CLink* m_psChildLink,
               CVector3 c_joint_axis,
               CVector3 c_linear_velocity,
               CVector3 c_angular_velocity,
               bool isEnabled);

         std::string GetId() { 
            return m_strId;
         }

         EType GetType() { 
            return m_eType;
         }

         CLinkEquippedEntity::CLink* GetParentLink() { 
            return m_psParentLink;
         }

         CLinkEquippedEntity::CLink* GetChildLink() { 
            return m_psChildLink;
         }

         CVector3 GetLinearVelocity() { 
            return m_cLinearVelocity;
         }

         CVector3 GetAngularVelocity() { 
            return m_cAngularVelocity;
         }

         CVector3 GetJointAxis() { 
            return m_cJointAxis;
         }

         bool GetEnabled() { 
            return m_bIsEnabled;
         }

         void SetActuatorParametersLinearVelocityX(Real f_target_velocity, bool b_isEnabled) {
            m_bIsEnabled = b_isEnabled;
            if (m_bIsEnabled == true) { 
               m_cLinearVelocity.SetX(f_target_velocity);
            } else { 
               m_cLinearVelocity.SetX(0.0f)
            }
         }

         void SetActuatorParametersLinearVelocityY(Real f_target_velocity, bool b_isEnabled) {
            m_bIsEnabled = b_isEnabled;
            if (m_bIsEnabled == true) { 
               m_cLinearVelocity.SetY(f_target_velocity);
            } else { 
               m_cLinearVelocity.SetY(0.0f)
            }
         }

         void SetActuatorParametersLinearVelocityZ(Real f_target_velocity, bool b_isEnabled) {
            m_bIsEnabled = b_isEnabled;
            if (m_bIsEnabled == true) { 
               m_cLinearVelocity.SetZ(f_target_velocity);
            } else { 
               m_cLinearVelocity.SetZ(0.0f)
            }
         }

         void SetActuatorParametersAngularVelocityX(Real f_target_velocity, bool b_isEnabled) {
            m_bIsEnabled = b_isEnabled;
            if (m_bIsEnabled == true) { 
               m_cAngularVelocity.SetX(f_target_velocity);
            } else { 
               m_cAngularVelocity.SetX(0.0f)
            }
         }

         void SetActuatorParametersAngularVelocityY(Real f_target_velocity, bool b_isEnabled) {
            m_bIsEnabled = b_isEnabled;
            if (m_bIsEnabled == true) { 
               m_cAngularVelocity.SetY(f_target_velocity);
            } else { 
               m_cAngularVelocity.SetY(0.0f)
            }
         }

         void SetActuatorParametersAngularVelocityZ(Real f_target_velocity, bool b_isEnabled) {
            m_bIsEnabled = b_isEnabled;
            if (m_bIsEnabled == true) { 
               m_cAngularVelocity.SetZ(f_target_velocity);
            } else { 
               m_cAngularVelocity.SetZ(0.0f)
            }
         }

         std::string m_strId;
         EType m_eType;
         CLinkEquippedEntity::CLink* m_psParentLink;
         CLinkEquippedEntity::CLink* m_psChildLink;
         CVector3 m_cLinearVelocity;
         CVector3 m_cAngularVelocity;
         CVector3 m_cJointAxis;
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

      virtual void UpdateComponents();

   protected:

      TJoints m_tJoints;

      TConfigurationNode m_tInitialConfiguration;
   };

}

#endif
