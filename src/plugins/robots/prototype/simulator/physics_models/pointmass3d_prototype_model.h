/**
 * @file <argos3/plugins/simulator/robots/foot-bot/simulator/pointmass3d/pointmass3d_prototype_model.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef POINTMASS3D_PROTOTYPE_H
#define POINTMASS3D_PROTOTYPE_H

namespace argos {
   class CPointMass3DEngine;
   class CPointMass3DPrototypeModel;
   class CFootBotEntity;
}

#include <argos3/plugins/simulator/physics_engines/pointmass3d/pointmass3d_model.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>

namespace argos {

   class CPointMass3DPrototypeModel : public CPointMass3DModel {

   public:
      
      CPointMass3DPrototypeModel(CPointMass3DEngine& c_engine,
                               CPrototypeEntity& c_prototype);

      virtual ~CPointMass3DPrototypeModel() {}
      
      virtual void Reset();

      // read in joint actuator settings
      virtual void UpdateFromEntityStatus();

      // perform physics step
      virtual void Step();

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual void CalculateBoundingBox();

   private:

      CPrototypeEntity& m_cPrototypeEntity;

      CPrototypeJointsDefaultActuator* m_pcJointsActuator;
   };

}

#endif
