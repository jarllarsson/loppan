#pragma once
#include <Artemis.h>
#include <btBulletDynamicsCommon.h>
#include <glm\gtc\type_ptr.hpp>
#include "TransformComponent.h"
#include <glm\gtc\matrix_transform.hpp>
#include <Util.h>

// =======================================================================================
//                                      ConstraintComponent
// =======================================================================================

///---------------------------------------------------------------------------------------
/// \brief	Brief
///        
/// # ConstraintComponent
/// 
/// 17-5-2014 Jarl Larsson
///---------------------------------------------------------------------------------------

class ConstraintComponent : public artemis::Component
{
public:
	struct ConstraintDesc
	{
		//glm::vec3 m_localAxis;
		//glm::vec3 m_parentLocalAxis;
		glm::vec3 m_localAnchor;
		glm::vec3 m_parentLocalAnchor;
		/*
		For each axis, if
		lower limit = upper limit
			The axis is locked
		lower limit < upper limit
			The axis is limited between the specified values
		lower limit > upper limit
			The axis is free and has no limits
		*/
		//glm::vec3 m_linearDOF_LULimits[2];
		glm::vec3 m_angularDOF_LULimits[2];
		bool m_collisionBetweenLinked;
	};


	ConstraintComponent(artemis::Entity* p_parent, const ConstraintDesc& p_desc)
	{
		m_parent = p_parent;
		m_constraint = NULL;
		m_desc = new ConstraintDesc(p_desc);
		m_inited = false;
		m_removed = false;
	}

	virtual ~ConstraintComponent()
	{
		delete m_desc;
		SAFE_DELETE(m_constraint);
	}


	const ConstraintDesc* getDesc() const;
	glm::vec3 getLowerLim() { return m_desc->m_angularDOF_LULimits[0]; }
	glm::vec3 getUpperLim() { return m_desc->m_angularDOF_LULimits[1]; }

	bool isInited();
	void init(btGeneric6DofConstraint* p_constraint, artemis::Entity* p_ownerEntity);

	btGeneric6DofConstraint* getConstraint();
	void forceRemove(btDiscreteDynamicsWorld* p_world);
	bool isRemoved() { return m_removed; }

	artemis::Entity* getParent();
	artemis::Entity* getOwnerEntity();
private:
	bool m_removed;
	bool m_inited;
	ConstraintDesc* m_desc;
	artemis::Entity* m_parent;
	artemis::Entity* m_owner; //< ie. the "owner"
	btGeneric6DofConstraint* m_constraint;
};
