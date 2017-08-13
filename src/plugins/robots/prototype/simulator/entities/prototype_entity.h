/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 * @author Thomas Varner - <thomas.g.varner@gmail.com>
 */

#ifndef PROTOTYPE_ENTITY_H
#define PROTOTYPE_ENTITY_H

namespace argos {
  class CPrototypeEntity;
  class CEmbodiedEntity;
  class CControllableEntity;
  class CLinkEquippedEntity;
  class CJointEquippedEntity;
  class CPrototypeJointsDefaultActuator;
  class CPrototypeJointsDefaultSensor;
  class CLEDEquippedEntity;
  class CProximitySensorEquippedEntity;
  class CLEDMedium;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CPrototypeEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CPrototypeEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Destroy();
      
      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CLinkEquippedEntity& GetLinkEquippedEntity() {
         return *m_pcLinkEquippedEntity;
      }

      inline CJointEquippedEntity& GetJointEquippedEntity() {
         return *m_pcJointEquippedEntity;
      }

      inline CPrototypeJointsDefaultActuator GetJointActuators() { 
        return *m_pcJointActuators;
      }

      inline CPrototypeJointsDefaultSensor GetJointSensors() { 
        return *m_pcJointSensors;
      }

      inline bool HasControllableEntity() const {
         return (m_pcControllableEntity != NULL);
      }

      inline CControllableEntity& GetControllableEntity() {
         ARGOS_ASSERT(m_pcLEDEquippedEntity != NULL,
                      "CPrototypeEntity::GetControllableEntity(), id=\"" <<
                      GetId() <<
                      "\": was not initialized with a controllable entity");
         return *m_pcControllableEntity;
      }

      inline CLEDEquippedEntity& GetLEDEquippedEntity() {
         ARGOS_ASSERT(m_pcLEDEquippedEntity != NULL,
                      "CPrototypeEntity::GetLEDEquippedEntity(), id=\"" <<
                      GetId() <<
                      "\": was not initialized with an LED equipped entity");
         return *m_pcLEDEquippedEntity;
      }

      inline CProximitySensorEquippedEntity& GetProximitySensorEquippedEntity() {
         ARGOS_ASSERT(m_pcProximitySensorEquippedEntity != NULL,
                      "CPrototypeEntity::GetProximitySensorEquippedEntity(), id=\"" <<
                      GetId() <<
                      "\": was not initialized with a proximity sensor equipped entity");
         return *m_pcProximitySensorEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "prototype";
      }

      inline CLinkEquippedEntity::SLink& GetReferenceLink() {
         return *m_pcReferenceLink;
      }

   private:
      CEmbodiedEntity* m_pcEmbodiedEntity;
      CLinkEquippedEntity* m_pcLinkEquippedEntity;
      CLinkEquippedEntity::SLink* m_psReferenceLink;
      CJointEquippedEntity* m_pcJointEquippedEntity;
      CPrototypeJointsDefaultActuator* m_pcJointActuators;
      CPrototypeJointsDefaultSensor* m_pcJointSensors;
      CLEDEquippedEntity* m_pcLEDEquippedEntity;
      CProximitySensorEquippedEntity* m_pcProximitySensorEquippedEntity;
      CLEDMedium* m_pcLEDMedium;
      CControllableEntity* m_pcControllableEntity;
   };

}

#endif
