/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_actuator.h>
 *
 * @brief This file provides the common interface definition of the joints actuator.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CCI_PROTOTYPE_JOINTS_ACTUATOR_H
#define CCI_PROTOTYPE_JOINTS_ACTUATOR_H

namespace argos {
   class CCI_PrototypeJointsActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/math/angles.h>

namespace argos {

   class CCI_PrototypeJointsActuator : virtual public CCI_Actuator {

   public:

      enum EActuatorAxis {
         LINEAR_X = CVector3(1, 0, 0),
         LINEAR_Y = CVector3(0, 1, 0),
         LINEAR_Z = CVector3(0, 0, 1),
         ANGULAR_X = CVector3(1, 0, 0),
         ANGULAR_Y = CVector3(0, 1, 0),
         ANGULAR_Z = CVector3(0, 0, 1)
      };

      class CJointActuator {
      public:
         CJointActuator() :
            m_bActuatorEnabled(false),
            m_fActuatorTargetVelocity(0.0f) {}
         void SetTargetVelocity(Real f_target_velocity) {
            m_fActuatorTargetVelocity = f_target_velocity;
         }
         Real GetTargetVelocity() {
            return m_fActuatorTargetVelocity;
         }
         void SetEnabled(bool b_enabled) {
            m_bActuatorEnabled = b_enabled;
         }
         bool GetEnabled() {
            return m_bActuatorEnabled;
         }
      protected:
         bool m_bActuatorEnabled;
         Real m_fActuatorTargetVelocity;
      };

   public:

      virtual ~CCI_PrototypeJointsActuator() {}

      virtual CJointActuator& GetJointActuator(std::string str_joint_id, EActuatorAxis e_axis) = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State*);
#endif

   };
}

#endif
