/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_ENTITY_H
#define PROTOTYPE_ENTITY_H

namespace argos {
   class CEmbodiedEntity;
   class CControllableEntity;
   class CLinkEquippedEntity;
   class CLinkEntity;
   class CJointEquippedEntity;
   class CLEDEquippedEntity;
   class CProximitySensorEquippedEntity;
   class CPrototypeEntity;
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

      virtual void UpdateComponents();
      
      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CLinkEquippedEntity& GetLinkEquippedEntity() {
         return *m_pcLinkEquippedEntity;
      }

      inline CJointEquippedEntity& GetJointEquippedEntity() {
         return *m_pcJointEquippedEntity;
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
                      "\": was not initialized with an proximity sensor equipped entity");
         return *m_pcProximitySensorEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "prototype";
      }

      inline CLinkEntity& GetReferenceLink() {
         return *m_pcReferenceLink;
      }

   private:
      CEmbodiedEntity* m_pcEmbodiedEntity;
      CControllableEntity* m_pcControllableEntity;
      CLinkEquippedEntity* m_pcLinkEquippedEntity;
      CJointEquippedEntity* m_pcJointEquippedEntity;
      CLEDEquippedEntity* m_pcLEDEquippedEntity;
      CProximitySensorEquippedEntity* m_pcProximitySensorEquippedEntity;

      CLinkEntity* m_pcReferenceLink;
   };

}

#endif
