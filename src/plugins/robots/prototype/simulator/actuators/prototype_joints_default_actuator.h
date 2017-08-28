/**
 * @file <argos3/plugins/robots/prototype/simulator/actuators/prototype_joints_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_JOINTS_DEFAULT_ACTUATOR_H
#define PROTOTYPE_JOINTS_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CPrototypeJointsDefaultActuator;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_actuator.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CPrototypeJointsDefaultActuator : public CSimulatedActuator,
                                           public CCI_PrototypeJointsActuator {

   public:
      class CSimulatedJointActuator : public CJointActuator {
      
      public:
         CSimulatedJointActuator(CJointEquippedEntity::Sjoint* pc_actuator_joint,
                                 EActuatorAxis e_actuator_axis) :
            m_pcActuatorJoint(pc_actuator_joint),
            m_cActuatorAxis(e_actuator_axis) {}
         
         EActuatorAxis GetAxis() {
            return m_cActuatorAxis;
         }
         
         CJointEquippedEntity::Sjoint& GetJoint() {
            return *m_pcActuatorJoint;
         }

      private:
         CJointEquippedEntity::Sjoint* m_pcActuatorJoint;
         CVector3 m_cActuatorAxis;
      };
      
   public:

      CPrototypeJointsDefaultActuator();
      virtual ~CPrototypeJointsDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual CJointActuator& GetJointActuator(std::string str_joint_id, CVector3 c_actuator_axis);

      virtual void Update();

      virtual void Reset();

   private:

      std::vector<std::vector<CSimulatedJointActuator> > m_vecActuators;

      CJointEquippedEntity* m_pcJointEquippedEntity;

   };
}

#endif
