/**
 * @file <argos3/plugins/robots/prototype/simulator/physics_models/dynamics3d_prototype_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_PROTOTYPE_MODEL_H
#define DYNAMICS3D_PROTOTYPE_MODEL_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>

namespace argos {

   class CDynamics3DPrototypeModel : public CDynamics3DMultiBodyObjectModel {

   public:

      CDynamics3DPrototypeModel(CDynamics3DEngine& c_engine,
                                CPrototypeEntity& c_entity);
      virtual ~CDynamics3DPrototypeModel();
      
      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

   protected:

      virtual btTransform GetModelCoordinates() const;

   private:

      CPrototypeEntity&       m_cPrototypeEntity;
      CLinkEquippedEntity&    m_cLinkEquippedEntity;
      CJointEquippedEntity&   m_cJointEquippedEntity;
   };
}

#endif
