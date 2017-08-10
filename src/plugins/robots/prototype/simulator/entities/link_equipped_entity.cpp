/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/link_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "link_equipped_entity.h"

namespace argos {

   SLink::SLink(const std::string& str_link_id,
               const EGeometry e_geometry,
               const Real f_mass,
               const CVector3 c_extents,
               const SAnchor& ps_Anchor) :
      m_strId(str_link_id),
      m_eGeometry(e_geometry),
      m_fMass(f_mass),
      m_cExtents(c_extents),
      m_psAnchor(ps_anchor) {
   }

   /****************************************/
   /****************************************/

   CLinkEquippedEntity::CLinkEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CLinkEquippedEntity::CLinkEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
          /* the second constructor is called so no need to call init on composable entity */
         // CComposableEntity::Init(t_tree);

         TConfigurationNodeIterator itLink("link");
         for(itLink = itLink.begin(&t_tree);
             itLink != itLink.end();
             ++itLink) {
            
            // get id
            std::string strLinkId;
            GetNodeAttribute(*itLink, 'id', strLinkId);

            // get mass
            Real fMass;
            GetNodeAttribute(*itLink, 'mass', fMass);

            // get offset position
            CVector3 cOffsetPosition;
            GetNodeAttributeOrDefault(*itLink, "position", cOffsetPosition, cOffsetPosition);

            // get orientation of the link wrt. the body
            CQuaternion cOffsetOrientation;
            GetNodeAttributeOrDefault(*itLink, "orientation", cOffsetOrientation, cOffsetOrientation);

            // get prototype embodied entity
            CEmbodiedEntity& cBody = GetParent().GetComponent<CEmbodiedEntity>("body");
            
            // create an anchor for each link, the origin has index 0, link 1 has index 1 and so on
            SAnchor& psAnchor = &(cBody.AddAnchor(strLinkId, cOffsetPosition, cOffsetOrientation));
            
            // Enable each anchor so that we can sync the entity's links with a physics engine
            psAnchor->Enable();

            /* Parse link geometry and dimensions */
            EGeometry eGeometry;
            CVector3 cExtents;
            GetNodeAttribute(*itLink, "geometry", strLinkGeometry);
            if(strLinkGeometry == "box") {
               eGeometry = BOX; 
               GetNodeAttribute(*itLink, "size", cExtents);
            } else if(strLinkGeometry == "cylinder") {
               eGeometry = CYLINDER;
               Real fRadius;
               Real fHeight;
               GetNodeAttribute(*itLink, "height", fHeight);
               GetNodeAttribute(*itLink, "radius", fRadius);
               cExtents.Set(fRadius * 2.0f, fRadius * 2.0f, fHeight);
            } else if(strLinkGeometry == "sphere") {
               eGeometry = SPHERE;
               Real fRadius;
               GetNodeAttribute(*itLink, "radius", fRadius);
               cExtents.Set(fRadius * 2.0f, fRadius * 2.0f, fRadius * 2.0f);
            } else {
               /* unknown geometry requested */
               THROW_ARGOSEXCEPTION("Geometry \"" << strLinkGeometry << "\" is not implemented");
            }

            SLink* psLink = new SLink(strLinkId,
                                       fMass,
                                       psAnchor,
                                       cExtents,
                                       eGeometry);
            m_tLinks[psLink->m_strId] = psLink;

            // if link is a base link, set it to the reference link on the prototype
            if (NodeAttributeExists(t_tree, 'base')) {
               // if base link already set, then throw an exception
               if (m_pcBase != NULL) { 
                  THROW_ARGOSEXCEPTION("Base link cannot be specified on more than one link. Previously on: " + m_pcBase->m_strId + ". Currently on: " + psLink->m_strId );
               }
               m_pcBase = psLink;
            } else if (itLink == itLink.end()) {
               // if no base link has been specified, throw an exception
               THROW_ARGOSEXCEPTION("Base link must be specified on at least one link");
            }

         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize link equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CLinkEquippedEntity);

   /****************************************/
   /****************************************/

}
