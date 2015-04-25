#pragma once

#include <windows.h>
#include <glm\gtc\type_ptr.hpp>

#include <InstanceData.h>
#include <CBuffers.h>
#include <Buffer.h>
#include <vector>
#include <Artemis.h>
#include <ResourceManager.h>


class Context;
class GraphicsDevice;
class TempController;
class Input;
class RenderSystem;
class RigidBodySystem;
class ControllerSystem;
class ControllerOptimizationSystem;
class Toolbar;
class DebugDrawer;
class DebugDrawBatch;
class SettingsData;

using namespace std;

// =======================================================================================
//                                      App
// =======================================================================================

///---------------------------------------------------------------------------------------
/// \brief	Brief
///        
/// # App
/// 
/// 18-4-2013 Jarl Larsson
///---------------------------------------------------------------------------------------

class App
{
public:
	App(HINSTANCE p_hInstance, unsigned int p_width=1280, unsigned int p_height=1024);
	virtual ~App();

	void run();
protected:
	Context* m_context;
	GraphicsDevice* m_graphicsDevice;
	DebugDrawer* m_debugDrawer;
	DebugDrawBatch* m_debugDrawBatch;

	void processInput();
	void handleContext(double p_dt, double p_physDt, unsigned int p_physSteps);
	void gameUpdate(double p_dt);

	void addOrderIndependentSystem(artemis::EntityProcessingSystem* p_system);

	void render();
private:
	enum CharCreateType
	{
		BIPED = 0, QUADRUPED = 1
	};

	enum InitExecSetup
	{
		SERIAL = 0, PARALLEL = 1
	};

	bool pumpMessage(MSG& p_msg);
	void processSystemCollection(vector<artemis::EntityProcessingSystem*>* p_systemCollection);
	void initFromSettings(SettingsData& p_settings);
	void drawDebugAxes();
	void drawDebugOptimizationGraphs(std::vector<double>* p_optimizationResults, float p_maxScore, float p_pmax, float p_pmin);
	static const double DTCAP;
	float m_fpsUpdateTick;

	void updateController(float p_dt);
	TempController*			m_controller;
	Input*					m_input;
	Toolbar*				m_toolBar;

	// Entity system handling
	artemis::World			m_world;
	vector<artemis::EntityProcessingSystem*> m_orderIndependentSystems;
	// Order dependant systems
	RigidBodySystem*		m_rigidBodySystem;
	RenderSystem*			m_renderSystem;
	ControllerSystem*		m_controllerSystem;
	ControllerOptimizationSystem*		m_optimizationSystem;

	CharCreateType m_characterCreateType;
	double m_time;
	float m_timeScale;
	bool m_timeScaleToggle;
	bool m_timePauseStepToggle;
	bool m_triggerPause;
	bool m_gravityStat, m_oldGravityStat;
	bool m_restart;
	bool m_saveParams;
	bool m_runOptimization;
	bool m_consoleMode;
	bool m_useToolbar;
	int m_measurementRuns;

	int   m_initWindowWidth, m_initWindowHeight;
	bool  m_initWindowMode;
	InitExecSetup   m_initExecSetup;
	int   m_initCharCountSerial;
	int   m_initParallelInvocCount;
	float m_initCharOffset;
	bool  m_measurePerf;
	int m_optmesSteps;
	double m_frameTime;

	std::vector<float>* m_bestParams;

	// Resource managers
	//ResourceManager<btCollisionShape> m_collisionShapes;

	
	//vector<glm::mat4> m_instanceOrigins;
	//Buffer<glm::mat4>* m_instances;
	Buffer<Mat4CBuffer>* m_vp;
};