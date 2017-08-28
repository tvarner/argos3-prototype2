/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 * @author Thomas Varner - <thomas.g.varner@gmail.com>
 */

#include "prototype_entity.h"

#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>

#include <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.h>

/*
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/electromagnet_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/tag_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/radio_equipped_entity.h>
*/

#include <argos3/plugins/robots/prototype/simulator/media/tag_medium.h>
#include <argos3/plugins/simulator/media/led_medium.h>

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeEntity::CPrototypeEntity() :
      CComposableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcLinkEquippedEntity(NULL),
      m_pcJointEquippedEntity(NULL),
      m_pcJointActuators(NULL),
      m_pcJointActuators(NULL),
      m_pcLEDEquippedEntity(NULL),
      m_pcProximitySensorEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CPrototypeEntity::Init(TConfigurationNode& t_tree) {
      try {
        /*
        * Init parent
        */
        CComposableEntity::Init(t_tree);

        /*
        * Create and init components
        */
        /* Embodied entity */
        m_pcEmbodiedEntity = new CEmbodiedEntity(this);
        m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
        AddComponent(*m_pcEmbodiedEntity);

        /* Link equipped entity*/
        m_pcLinkEquippedEntity = new CLinkEquippedEntity(this, 'links');
        m_pcLinkEquippedEntity->Init(GetNode(t_tree, "links"));
        AddComponent(*m_pcLinkEquippedEntity);

        /* Reference link */
        m_psReferenceLink = m_pcLinkEquippedEntity->GetBase();

        /* Joint equipped entity */
        if(NodeExists(t_tree, "joints")) {
          m_pcJointEquippedEntity = new CJointEquippedEntity(this, 'joints');
          m_pcJointEquippedEntity->Init(GetNode(t_tree, "joints"));
          AddComponent(*m_pcJointEquippedEntity);

          /* Joint actuators */
          m_pcJointActuators = new CPrototypeJointsDefaultActuator();
          m_pcJointActuators->SetRobot(this);

          /* Joint sensors */
          m_pcJointSensors = new CPrototypeJointsDefaultSensor();
          m_pcJointSensors->SetRobot(this);
        }

        /***********************/
        /********DEVICES********/
        /***********************/

        if(NodeExists(t_tree, "devices")) {
          TConfigurationNodeIterator itDevice;
          for(itDevice = itDevice.begin(&GetNode(t_tree, "devices"));
            itDevice != itDevice.end();
              ++itDevice) {
              if(itDevice->Value() == "rangefinders") {
                m_pcProximitySensorEquippedEntity = new CProximitySensorEquippedEntity(this);
                m_pcProximitySensorEquippedEntity->Init(*itDevice);
                AddComponent(*m_pcProximitySensorEquippedEntity);
              }
              else if(itDevice->Value() == "leds" ) {

                /* Init LED equipped entity component */
                m_pcLEDEquippedEntity = new CLEDEquippedEntity(this);
                AddComponent(*m_pcLEDEquippedEntity);

                /* Create LED equipped entity
                 * NOTE: the LEDs are not added to the medium yet
                 */
                m_pcLEDEquippedEntity->Init(*itDevice);
                /* Add the LEDs to the medium */
                std::string strMedium;
                GetNodeAttribute(*itDevice, "medium", strMedium);
                m_pcLEDMedium = &CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium);
                m_pcLEDEquippedEntity->AddToMedium(*m_pcLEDMedium);
              }
              /*
              else if(itDevice->Value() == "cameras" ) {
                CCameraEquippedEntity* m_pcEquippedEntity =
                   new CCameraEquippedEntity(this);
                m_pcEquippedEntity->Init(*itDevice);
                AddComponent(*m_pcEquippedEntity);
              }
              else if(itDevice->Value() == "electromagnets" ) {
                CElectromagnetEquippedEntity* m_pcEquippedEntity =
                   new CElectromagnetEquippedEntity(this);
                m_pcEquippedEntity->Init(*itDevice);
                AddComponent(*m_pcEquippedEntity);
              }
              else if(itDevice->Value() == "tags" ) {
                CTagEquippedEntity* m_pcEquippedEntity =
                   new CTagEquippedEntity(this);
                m_pcEquippedEntity->Init(*itDevice);
                std::string strMedium;
                GetNodeAttribute(*itDevice, "medium", strMedium);
                m_pcEquippedEntity->AddToMedium(CSimulator::GetInstance().GetMedium<CTagMedium>(strMedium));
                AddComponent(*m_pcEquippedEntity);
              }
              else if(itDevice->Value() == "radios" ) {
                CRadioEquippedEntity* m_pcEquippedEntity =
                  new CRadioEquippedEntity(this);
                m_pcEquippedEntity->Init(*itDevice);
                AddComponent(*m_pcEquippedEntity);
              }
          */
            else {
              THROW_ARGOSEXCEPTION("Device type \"" << itDevice->Value() << "\" not implemented");
            }
          }
        }

        /* Controllable entity
        It must be the last one, for actuators/sensors to link to composing entities correctly */
        if(NodeExists(t_tree, "controller")) {
          m_pcControllableEntity = new CControllableEntity(this);
          AddComponent(*m_pcControllableEntity);
          m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
        }

        /* Update components */
        UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
    }

    /****************************************/
    /****************************************/

    void CPrototypeEntity::Destroy() { 
      //@todo: destroy this entity
    }

    /****************************************/
    /****************************************/

    REGISTER_ENTITY(CPrototypeEntity,
                   "prototype",
                   "1.1",
                   "Michael Allwright [allsey87@gmail.com],
                    Thomas Varner [thomas.g.varner@gmail.com]",
                   "A generic and configurable entity",
                   "[long description]",
                   "Under development"
    );

    /****************************************/
    /****************************************/

    REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CPrototypeEntity);

    /****************************************/
    /****************************************/
}
