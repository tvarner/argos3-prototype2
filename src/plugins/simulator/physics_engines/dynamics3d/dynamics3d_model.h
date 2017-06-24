/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_MODEL_H
#define DYNAMICS3D_MODEL_H

namespace argos {
   class  CDynamics3DEngine;
}

#include "dynamics3d_engine.h"

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {

   /****************************************/
   /****************************************/

   /*
   inline CVector3 BulletToARGoS(const btVector3& c_bt_vector) {
      return CVector3(c_bt_vector.getX(), -c_bt_vector.getZ(), c_bt_vector.getY());
   }
   
   inline btVector3 ARGoSToBullet(const CVector3& c_a_vector) {
      return btVector3(c_a_vector.GetX(), c_a_vector.GetZ(), -c_a_vector.GetY());
   }
   
   inline CQuaternion BulletToARGoS(const btQuaternion& c_bt_quaternion) {
      return CQuaternion(c_bt_quaternion.getW(), c_bt_quaternion.getX(),
                         -c_bt_quaternion.getZ(), c_bt_quaternion.getY());
   }
   
   inline btQuaternion ARGoSToBullet(const CQuaternion& c_a_quaternion) {
      return btQuaternion(c_a_quaternion.GetX(), c_a_quaternion.GetZ(), 
                          -c_a_quaternion.GetY(), c_a_quaternion.GetW());
   }
   */

   /****************************************/
   /****************************************/

   class CDynamics3DModel : public CPhysicsModel {

   public:

      typedef std::map<std::string, CDynamics3DModel*> TMap;
      
   public:

      CDynamics3DModel(CDynamics3DEngine& c_engine,
                       CEmbodiedEntity& c_entity) :
         CPhysicsModel(c_engine, c_entity),
         m_cEngine(c_engine) {}

      virtual ~CDynamics3DModel() {}

      virtual void Reset() = 0;

      /**
       * Returns the dynamics 3D engine.
       * @return The dynamics 3D engine.
       */
      inline CDynamics3DEngine& GetEngine() {
         return m_cEngine;
      }

      /**
       * Returns the dynamics 3D engine state as a const reference.
       * @return The dynamics 3D engine state as a const reference.
       */
      inline const CDynamics3DEngine& GetEngine() const {
         return m_cEngine;
      }

   protected:

      CDynamics3DEngine& m_cEngine;

   };
   
   /****************************************/
   /****************************************/

}

#endif
