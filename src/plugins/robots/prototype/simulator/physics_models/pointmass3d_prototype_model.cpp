/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "pointmass3d_footbot_model.h"
#include <argos3/core/utility/math/cylinder.h>

namespace argos {

   static const Real FOOTBOT_RADIUS                   = 0.085036758f;
   static const Real FOOTBOT_INTERWHEEL_DISTANCE      = 0.14f;
   static const Real FOOTBOT_HEIGHT                   = 0.146899733f;

   enum FOOTBOT_WHEELS {
      FOOTBOT_LEFT_WHEEL = 0,
      FOOTBOT_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   CPointMass3DProtoTypeModel::CPointMass3DProtoTypeModel(CPointMass3DEngine& c_engine,
                                                      CProtoTypeEntity& c_prototype) :
      CPointMass3DModel(c_engine, c_prototype.GetEmbodiedEntity()),
      m_cPrototypeEntity(c_prototype) {
   }

   /****************************************/
   /****************************************/

   void CPointMass3DProtoTypeModel::Reset() {
      // @todo: set initial position and orientation for each link and joint

      // apply FK to calculate position and orientation of each link and joint
         // traverse link and joints from base
   }

   /****************************************/
   /****************************************/


   void CPointMass3DProtoTypeModel::UpdateFromEntityStatus() {
      // No need to implement since updating joint linear and angular velocities 
      // directly in actuator update method.

      // I assume PID control goes here for joint transform update
         // update controls by servo gain * error
   }

   /****************************************/
   /****************************************/

   void CPointMass3DProtoTypeModel::Step() {
      // @todo: 

      // update joint position and orientation from updated angular velocity

      // apply FK to calculate position and orientation of each link and joint
         // traverse link and joints from base
   }

   /****************************************/
   /****************************************/

   void CPointMass3DProtoTypeModel::CalculateBoundingBox() {
      // @todo: calculate bounding box for each link given position and
      // orientation of each link
   }

   /****************************************/
   /****************************************/

   bool CPointMass3DProtoTypeModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      // @todo: calculate intersection with each link
/*
      CCylinder m_cShape(FOOTBOT_RADIUS,
                         FOOTBOT_HEIGHT,
                         m_cPosition,
                         CVector3::Z);
      bool bIntersects = m_cShape.Intersects(f_t_on_ray, c_ray);
      return bIntersects;
*/
   }

   /****************************************/
   /****************************************/

   void CPointMass3DProtoTypeModel::UpdateBaseAnchor(SAnchor& s_anchor) {

   }

   void CPointMass3DProtoTypeModel::UpdateLinkAnchor(SAnchor& s_anchor) {

   }

   /****************************************/
   /****************************************/
   
   REGISTER_STANDARD_POINTMASS3D_OPERATIONS_ON_ENTITY(CProtoTypeEntity, CPointMass3DProtoTypeModel);

   /****************************************/
   /****************************************/

}
