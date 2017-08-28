/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "pointmass3d_footbot_model.h"
#include <argos3/core/utility/math/cylinder.h>

namespace argos {

   	/****************************************/
   	/****************************************/

   	CPointMass3DProtoTypeModel::CPointMass3DProtoTypeModel(CPointMass3DEngine& c_engine,
													  CProtoTypeEntity& c_prototype) :
	  	CPointMass3DModel(c_engine, c_prototype.GetEmbodiedEntity()),
	  	m_cPrototypeEntity(c_prototype) {
			m_pcJointsActuator = GetActuator<CPrototypeJointsDefaultActuator>("joints");
	  	}

   	/****************************************/
   	/****************************************/

   	void CPointMass3DProtoTypeModel::Reset() {
	  	// @todo: set initial position and orientation for each link and joint
   	}

   	/****************************************/
   	/****************************************/


   	/*
		Update physics model 
   	*/
   	/*
		NOTE: 	no need to implement this method since joint linear and angular
				velocities are updated directly the in the joint actuator update
				methods.
				(see PrototypeJointsDefaultActuator::Update() <-- this method is called in UpdateControllableEntitiesAct())
   	*/
   	void CPointMass3DProtoTypeModel::UpdateFromEntityStatus() {}

   	/****************************************/
   	/****************************************/

   	void CPointMass3DProtoTypeModel::Step() {
		// @todo:

		// update joint position and orientation given updated linear and angular velocities
   		TraverseFKBaseLink();
  	}

  	void CPointMass3DProtoTypeModel::TraverseFKBaseLink() { 

   		// get child joint

   		// traverse base child joint		
	   	TraverseFKJoint(...)
  	}

  	void CPointMass3DProtoTypeModel::TraverseFKLink(CLinkEquippedEntity::CLink c_link) { 

  		// 

  		// for each child 
  		TraverseFKJoint();
  	}

  	void CPointMass3DProtoTypeModel::TraverseFKJoint(CLinkEquippedEntity::CLink c_link) { 

	   	// get joint linear velocity

	   	// get joint angular velocity

  		// for each child link

	  		// compute local transform of joint from child position

	 		// update local transform given linear and angular velocities

  			// 

	  			// compute rotation matrix according to axis of joint

	  				// if joint type is spherical, revolute, or continuous

	  					// transform matrix = get quaternion from joint axis and angle

	  					// transform matrix = normalize the quaternion

	  				// else if joint type is prismatic

	  					/* only joint translation matters */

	  					// transform matrix = translation matrix from joint axis and angle

	  				// else joint is fixed

	  					// transform matrix = identity matrix

	  		// local transform = matrixMultiply(local transform, )

	  		// compute world transform of joint


  		// for each child 
  		TraverseFKLink();
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
   
   	REGISTER_STANDARD_POINTMASS3D_OPERATIONS_ON_ENTITY(CProtoTypeEntity, CPointMass3DProtoTypeModel);

   	/****************************************/
   	/****************************************/
}
