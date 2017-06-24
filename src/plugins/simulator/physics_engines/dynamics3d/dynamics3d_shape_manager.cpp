#include "dynamics3d_shape_manager.h"
#include <argos3/core/utility/configuration/argos_exception.h>
//#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   std::vector<CDynamics3DShapeManager::SBoxResource> CDynamics3DShapeManager::m_vecBoxResources;

   /****************************************/
   /****************************************/
   
   btBoxShape* CDynamics3DShapeManager::RequestBox(const btVector3& c_half_extents) {
      std::vector<SBoxResource>::iterator itBoxResource;      
      for(itBoxResource = m_vecBoxResources.begin();
          itBoxResource != m_vecBoxResources.end();
          ++itBoxResource) {
         if(itBoxResource->m_cHalfExtents == c_half_extents) break;
      }      
      // if it doesn't exist, create a new one
      if(itBoxResource == m_vecBoxResources.end()) {
         itBoxResource = m_vecBoxResources.insert(itBoxResource, 
                                            SBoxResource(c_half_extents, new btBoxShape(c_half_extents)));
      }
      itBoxResource->m_unInUseCount++;
      return itBoxResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DShapeManager::ReleaseBox(const btBoxShape* pc_release) {
      std::vector<SBoxResource>::iterator itBoxResource;      
      for(itBoxResource = m_vecBoxResources.begin();
          itBoxResource != m_vecBoxResources.end();
          ++itBoxResource) {
         if(itBoxResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itBoxResource == m_vecBoxResources.end()) {
         THROW_ARGOSEXCEPTION("Box not found");
      }
      itBoxResource->m_unInUseCount--;
      if(itBoxResource->m_unInUseCount == 0) {
         delete itBoxResource->m_cShape;
         m_vecBoxResources.erase(itBoxResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DShapeManager::SBoxResource::SBoxResource(const btVector3& c_half_extents,
                                                       btBoxShape* c_shape) : 
      m_cHalfExtents(c_half_extents),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/
}

