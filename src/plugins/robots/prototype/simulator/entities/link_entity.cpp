/**
 * file <argos3/plugins/robots/prototype/simulator/entities/link_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "link_entity.h"

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CLinkEntity::CLinkEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent),
      m_fMass(0.0f) {}

   /****************************************/
   /****************************************/



   void CLinkEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CEntity::Init(t_tree);
         /* Parse link geometry and dimensions */
         std::string strLinkGeometry;
         GetNodeAttribute(t_tree, "geometry", strLinkGeometry);
         if(strLinkGeometry == "box") {
            m_eGeometry = BOX; 
            GetNodeAttribute(t_tree, "size", m_cExtents);
         } else if(strLinkGeometry == "cylinder") {
            m_eGeometry = CYLINDER;
            Real fRadius;
            Real fHeight;
            GetNodeAttribute(t_tree, "height", fHeight);
            GetNodeAttribute(t_tree, "radius", fRadius);
            m_cExtents.Set(fRadius * 2.0f, fRadius * 2.0f, fHeight);
         } else if(strLinkGeometry == "sphere") {
            m_eGeometry = SPHERE;
            Real fRadius;
            GetNodeAttribute(t_tree, "radius", fRadius);
            m_cExtents.Set(fRadius * 2.0f, fRadius * 2.0f, fRadius * 2.0f);
         } else {
            /* unknown geometry requested */
            THROW_ARGOSEXCEPTION("Geometry \"" << strLinkGeometry << "\" is not implemented");
         }
         /* Parse link geometry and dimensions */
         GetNodeAttribute(t_tree, "mass", m_fMass);
         /* Get the position of the link wrt. the body */
         CVector3 cOffsetPosition;
         GetNodeAttributeOrDefault(t_tree, "position", cOffsetPosition, cOffsetPosition);
         /* Get the orientation of the link wrt. the body */
         CQuaternion cOffsetOrientation;
         GetNodeAttributeOrDefault(t_tree, "orientation", cOffsetOrientation, cOffsetOrientation);
         /* create an anchor for this link */
         // TODO do we really need to be inherited from a positional entity? No. The anchor stores the location of this link in the GCS
         // TODO do we need to store a pointer to the anchor in this class? Yes. Since we don't use the positional entity, the anchor contains the position and orientation of this link
         // TODO if we merge this functionality into link equipped entity and create a struct, we can just use it's constructor to init the reference to the anchor
         /* Get a link to the emboddied entity */
         CEmbodiedEntity& cBody = GetParent().GetParent().GetComponent<CEmbodiedEntity>("body");
         // Note if we move to a simple struct, we won't need to call get parent twice
         if(GetId() != static_cast<CPrototypeEntity&>(GetParent().GetParent()).m_strReferenceLink) {
            m_psAnchor = &(cBody.AddAnchor(GetId(), cOffsetPosition, cOffsetOrientation));
         }
         else {
            m_psAnchor = &(cBody.GetOriginAnchor());
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing link entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CLinkEntity::Reset() {
   }

   /****************************************/
   /****************************************/

   void CLinkEntity::Destroy() {
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CLinkEntity);

   /****************************************/
   /****************************************/

}
