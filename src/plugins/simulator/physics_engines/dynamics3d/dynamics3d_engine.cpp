/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_engine.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>

#include <algorithm>

namespace argos {

   /****************************************/
   /****************************************/   

   //btStaticPlaneShape CDynamics3DEngine::m_cGroundCollisionShape();

   /****************************************/
   /****************************************/

   void PluginPreTickCallback(btDynamicsWorld* pc_world, btScalar f_time_step) {
      // CDynamics3DEngine* pc_engine = static_cast<CDynamics3DEngine*>(pc_world->getWorldUserInfo());
      //pc_engine->UpdatePhysicsPlugins(f_time_step);
   }

   /****************************************/
   /****************************************/

   CDynamics3DEngine::CDynamics3DEngine() :
      m_pcRNG(NULL),
      m_cRandomSeedRange(0,1000),
      m_pcBroadphaseInterface(NULL),
      m_pcCollisionConfiguration(NULL),
      m_pcCollisionDispatcher(NULL),
      m_pcSolver(NULL),
      m_pcWorld(NULL),
      m_cGroundShape(btVector3(0.0f, 1.0f, 0.0f), 0),
      m_cGround(0, &m_cGroundState, &m_cGroundShape),     
      m_unIterations(10) {}

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Init(TConfigurationNode& t_tree) {
      /* Init parent */
      CPhysicsEngine::Init(t_tree);
      /* create the random number generator */
      m_pcRNG = CRandom::CreateRNG("argos");      
      /* Parse the XML */
      GetNodeAttributeOrDefault(t_tree, "iterations", m_unIterations, m_unIterations);
      m_fDeltaT = GetPhysicsClockTick() / static_cast<btScalar>(m_unIterations);
      /* Select the default broadphase, collision configuration, dispatcher and solver */
      m_pcBroadphaseInterface = new btDbvtBroadphase;
      m_pcCollisionConfiguration = new btDefaultCollisionConfiguration;
      m_pcCollisionDispatcher = new btCollisionDispatcher(m_pcCollisionConfiguration);
      m_pcSolver = new btMultiBodyConstraintSolver;
      /* Create the physics world */
      m_pcWorld = new btMultiBodyDynamicsWorld(m_pcCollisionDispatcher,
                                               m_pcBroadphaseInterface,
                                               m_pcSolver,
                                               m_pcCollisionConfiguration);
      /* clear the forces in the world (shouldn't  be required as there are no bodies in the world) */
      m_pcWorld->clearForces();
      /* reset the solvers and dispatchers */
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
      m_pcSolver->setRandSeed(m_pcRNG->Uniform(m_cRandomSeedRange));
      /* Add a static plane as the experiment floor on request */
      if(NodeExists(t_tree, "floor")) {
         m_pcWorld->addRigidBody(&m_cGround);
      }
      /* load the plugins */
      /*
      if(NodeExists(t_tree, "plugins")) {
         TConfigurationNodeIterator itPlugin;
         for(itPlugin = itPlugin.begin(&GetNode(t_tree, "plugins"));
             itPlugin != itPlugin.end();
             ++itPlugin) {
            CDynamics3DPlugin* pcPlugin = CFactory<CDynamics3DPlugin>::New(itPlugin->Value());
            pcPlugin->SetEngine(*this);
            pcPlugin->Init(*itPlugin);
            AddPhysicsPlugin(*pcPlugin);
         }
         m_pcWorld->setInternalTickCallback(PluginPreTickCallback, static_cast<void*>(this), true);
      }
      */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Reset() {
      /* Remove and reset the physics entities
       * by iterating over the vector, we ensure that the entities are removed in the same order
       * as they were added during initisation
       */
      for(CDynamics3DModel::TMap::iterator itModel = m_tPhysicsModels.begin();
          itModel != m_tPhysicsModels.end();
          ++itModel) {
         /* remove model from plugins */
         /*
         for(CDynamics3DPlugin::TMap::iterator itPlugin = m_tPhysicsPlugins.begin();
             itPlugin != m_tPhysicsPlugins.end();
             ++itPlugin) {
            itPlugin->second->UnregisterModel(*itModel->second);
         }
         */
         itModel->second->Reset();
      }
      /*
      if(m_pcGround != NULL) {
         m_pcGround->RemoveBodyFromWorld(m_pcWorld);
         m_pcGround->Reset();
      }
      */
      /* clear the forces in the world (shouldn't be required as there are no bodies in the world) */
      m_pcWorld->clearForces();
      /* reset the solvers and dispatchers */
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
      // TODO this seems wrong
      m_pcSolver->reset();

      m_pcSolver->setRandSeed(m_pcRNG->Uniform(m_cRandomSeedRange));
      /* reset plugins */
      /*
      for(CDynamics3DPlugin::TMap::iterator itPlugin = m_tPhysicsPlugins.begin();
          itPlugin != m_tPhysicsPlugins.end();
          ++itPlugin) {
         (*itPlugin)->Reset();
      } 
      */
      /* Add elements back to the engine
       * by iterating over the vector, we ensure that the entities are added in the same order
       * as they were added during initialization, this is important for repeatability between resets
       */
      /*
      if(m_pcGround != NULL) {
         m_pcGround->AddBodyToWorld(m_pcWorld);
      }
      */
      for(CDynamics3DModel::TMap::iterator itModel = m_tPhysicsModels.begin();
          itModel != m_tPhysicsModels.end();
          ++itModel) {
         /* add model to plugins */
         /*
         for(CDynamics3DPlugin::TMap::iterator itPlugin = m_tPhysicsPlugins.begin();
             itPlugin != m_tPhysicsPlugins.end();
             ++itPlugin) {
            (*itPlugin)->RegisterModel(**itModel);
         }
         */
      }
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Destroy() {
      /* empty the physics entity map */
      for(CDynamics3DModel::TMap::iterator it = m_tPhysicsModels.begin();
          it != m_tPhysicsModels.end();
          ++it) {
         delete it->second;
      }
      m_tPhysicsModels.clear();      
      /* remove the floor if it was added */
      /*
      if(m_pcGround != NULL) {
         m_pcGround->RemoveBodyFromWorld(m_pcWorld);
         delete m_pcGround;
      }
      */
      /* delete the dynamics world */
      delete m_pcWorld;
      delete m_pcSolver;
      delete m_pcCollisionDispatcher;
      delete m_pcCollisionConfiguration;
      delete m_pcBroadphaseInterface;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Update() {      
      /* Update the physics state from the entities */
      for(CDynamics3DModel::TMap::iterator it = m_tPhysicsModels.begin();
          it != m_tPhysicsModels.end(); ++it) {
         it->second->UpdateFromEntityStatus();
      }
      /* Step the simuation forwards */
      m_pcWorld->stepSimulation(GetPhysicsClockTick(), m_unIterations, m_fDeltaT);
      /* Update the simulated space */
      for(CDynamics3DModel::TMap::iterator it = m_tPhysicsModels.begin();
          it != m_tPhysicsModels.end(); ++it) {
         it->second->UpdateEntityStatus();
      }
   }
   
   /****************************************/
   /****************************************/
   
   void CDynamics3DEngine::CheckIntersectionWithRay(TEmbodiedEntityIntersectionData& t_data,
                                                    const CRay3& c_ray) const {
      /* convert the start and end ray vectors to the bullet coordinate system */
      btVector3 cRayStart(c_ray.GetStart().GetX(), c_ray.GetStart().GetZ(), -c_ray.GetStart().GetY());
      btVector3 cRayEnd(c_ray.GetEnd().GetX(), c_ray.GetEnd().GetZ(), -c_ray.GetEnd().GetY());
      btCollisionWorld::ClosestRayResultCallback cResult(cRayStart, cRayEnd);
      /* the default flag/algorithm 'kF_UseSubSimplexConvexCastRaytest' is too approximate for our scale */
      cResult.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
      m_pcWorld->rayTest(cRayStart, cRayEnd, cResult);
      if (cResult.hasHit()) {
         Real f_t = (cResult.m_hitPointWorld - cRayStart).length() / c_ray.GetLength();
         CDynamics3DModel* pcModel = static_cast<CDynamics3DModel*>(cResult.m_collisionObject->getUserPointer());
         /* required as some bodies do not belong to models (i.e. the floor) */
         t_data.push_back(SEmbodiedEntityIntersectionItem(&(pcModel->GetEmbodiedEntity()), f_t));
      }
   }

   /****************************************/
   /****************************************/
   
   size_t CDynamics3DEngine::GetNumPhysicsModels() {
      return m_tPhysicsModels.size();
   }
     
   /****************************************/
   /****************************************/

   bool CDynamics3DEngine::AddEntity(CEntity& c_entity) {
      SOperationOutcome cOutcome =
         CallEntityOperation<CDynamics3DOperationAddEntity, CDynamics3DEngine, SOperationOutcome>
         (*this, c_entity);
      return cOutcome.Value;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DEngine::RemoveEntity(CEntity& c_entity) {
      SOperationOutcome cOutcome =
         CallEntityOperation<CDynamics3DOperationRemoveEntity, CDynamics3DEngine, SOperationOutcome>
         (*this, c_entity);
      return cOutcome.Value;
   }
  
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddPhysicsModel(const std::string& str_id,
                                           CDynamics3DModel& c_model) {
      m_tPhysicsModels[str_id] = &c_model;
      /* Notify the plugins of the added model */
      /*
      for(CDynamics3DPlugin::TMap::iterator itPlugin = m_tPhysicsPlugins.begin();
          itPlugin != m_tPhysicsPlugins.end();
          ++itPlugin) {
         (*itPlugin)->RegisterModel(c_model);
      }
      */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemovePhysicsModel(const std::string& str_id) {
      CDynamics3DModel::TMap::iterator it = m_tPhysicsModels.find(str_id);
      if(it != m_tPhysicsModels.end()) {
         /* Notify the plugins of model removal */
         /*
         for(CDynamics3DPlugin::TMap::iterator itPlugin = m_tPhysicsPlugins.begin();
             itPlugin != m_tPhysicsPlugins.end();
             ++itPlugin) {
            (*itPlugin)->UnregisterModel(**it);
         }
         */
         delete it->second;
         m_tPhysicsModels.erase(it);
      }
      else {
         THROW_ARGOSEXCEPTION("Dynamics3D model id \"" << str_id << "\" not found in dynamics 3D engine \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   /*
   void CDynamics3DEngine::AddPhysicsPlugin(const std::string& str_id,
                                            CDynamics3DPlugin& c_plugin) {
      m_tPhysicsPlugins[str_id] = &c_model;
   }
   */

   /****************************************/
   /****************************************/

   /*
   void CDynamics3DEngine::RemovePhysicsPlugin(const std::string& str_id) {
      CDynamics3DPlugin::TMap::iterator it = m_tPhysicsPlugins.find(str_id);
      if(it != m_tPhysicsPlugins.end()) {
         delete it->second;
         m_tPhysicsPlugins.erase(it);
      }
      else {
         THROW_ARGOSEXCEPTION("Dynamics3D plugin id \"" << str_id << "\" not found in dynamics 3D engine \"" << GetId() << "\"");
      }
   }
   */

   /****************************************/
   /****************************************/

   /*
   void CDynamics3DEngine::UpdatePhysicsPlugins(btScalar f_time_step) {
      m_pcWorld->clearForces();
      for(CDynamics3DPlugin::TMap::iterator it = m_tPhysicsPlugins.begin();
          it != m_tPhysicsPlugins.end();
          ++it) {
         (*it)->Update();
      }
   }
   */

   /****************************************/
   /****************************************/   
  

   REGISTER_PHYSICS_ENGINE(CDynamics3DEngine,
                           "dynamics3d",
                           "Michael Allwright [allsey87@gmail.com]",
                           "1.0",
                           "A 3D dynamics physics engine",
                           "Dynamics3D is a plugin based on the open source bullet physics library",
                           "Under development");
}
