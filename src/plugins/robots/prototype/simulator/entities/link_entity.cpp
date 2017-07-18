/**
 * file <argos3/plugins/robots/prototype/simulator/entities/link_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "link_entity.h"

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CLinkEntity::CLinkEntity(CComposableEntity* pc_parent) :
      CPositionalEntity(pc_parent),
      m_fMass(0.0f) {}

   /****************************************/
   /****************************************/

   void CLinkEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CPositionalEntity::Init(t_tree);
         /* Get a link to the emboddied entity */
         CEmbodiedEntity& cBody = GetParent().GetComponent<CEmbodiedEntity>("body");
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
         /* create an anchor for this link */
         // TODO do we really need to be inherited from a positional entity?
         // TODO do we need to store a pointer to the anchor in this class?
         m_psAnchor = &(cBody.AddAnchor(GetId(), GetPosition(), GetOrientation()));
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
