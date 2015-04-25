#pragma once
#include <glm\gtc\type_ptr.hpp>
// =======================================================================================
//                                      IK2Handler
// =======================================================================================

///---------------------------------------------------------------------------------------
/// \brief	Handler and solver for 2-link IK setup
///        
/// # IK2Handler
/// 
/// 11-7-2014 Jarl Larsson
///---------------------------------------------------------------------------------------
class DebugDrawBatch;
class IK2Handler
{
public:
	IK2Handler(int p_kneeFlip=1);
	~IK2Handler();


	void solve(const glm::vec3& p_footPos, const glm::vec3& p_upperLegJointPos, float p_upperLegLen, float p_lowerLegLen, DebugDrawBatch* p_drawer);

	float getUpperLegAngle() const;
	float getLowerLocalLegAngle() const;
	float getLowerWorldLegAngle() const;
	int getKneeFlip() const;

	glm::vec3& getHipPos();
	glm::vec3& getKneePos();
	glm::vec3& getFootPos();
protected:
	void debugDraw(DebugDrawBatch* p_drawer);
private:
	int m_kneeFlip;
	void updateAngles(float p_lowerAngle, float p_upperAngle);
	float m_upperAngle;
	float m_lowerAngle;
	float m_lowerAngleW;
	glm::vec3 m_hipPos;
	glm::vec3 m_kneePos;
	glm::vec3 m_endPos;
};