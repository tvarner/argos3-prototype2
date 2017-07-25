/*sudo*
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_box_model.h"
#include "dynamics3d_shape_manager.h"
#include "dynamics3d_engine.h"


namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBoxModel::CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                                            CBoxEntity& c_box) :
      CDynamics3DSingleBodyObjectModel(c_engine, c_box) {
      /* Fetch a collision shape for this model */
      m_pcShape = CDynamics3DShapeManager::RequestBox(
         btVector3(c_box.GetSize().GetX() * 0.5f,
                   c_box.GetSize().GetZ() * 0.5f, 
                   c_box.GetSize().GetY() * 0.5f));
      /* calculate the geometric offset */
      m_cCenterOfMassOffset = btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -c_box.GetSize().GetZ() * 0.5f, 0.0f));
      /* calculate the mass */
      m_fMass = c_box.GetEmbodiedEntity().IsMovable() ? c_box.GetMass() : 0.0f;
      /* calculate the inertia */
      if(m_fMass != 0.0f && m_pcShape != NULL) {
         m_pcShape->calculateLocalInertia(m_fMass, m_cInertia);
      }
      SetBody();
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DBoxModel::~CDynamics3DBoxModel() {
      CDynamics3DShapeManager::ReleaseBox(static_cast<btBoxShape*>(m_pcShape));
   }
   
   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxModel);

}
