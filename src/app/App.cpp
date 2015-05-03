#include "App.h"
#include <DebugPrint.h>
#include "DebugDrawer.h" // DirectXTK stuff must be included before bullet stuff.. X(
#include "DebugDrawBatch.h"
#include "AdvancedEntitySystem.h"
#include <Context.h>
#include <ContextException.h>

#include <GraphicsDevice.h>
#include <GraphicsException.h>
#include <BufferFactory.h>

#include <Input.h>
#include <Util.h>
#include <MeasurementBin.h>
#include <MathHelp.h>

#include <ValueClamp.h>
#include "TempController.h"

#include <iostream> 

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>


#include "PositionSystem.h"
#include "RigidBodySystem.h"
#include "RenderSystem.h"
#include "ControllerSystem.h"
#include "PhysicsWorldHandler.h"
#include "Toolbar.h"

#include "ConstantForceComponent.h"
#include "ConstantForceSystem.h"
#include "Time.h"
#include "PhysWorldDefines.h"
#include "PositionRefSystem.h"
#include "ConstraintSystem.h"
#include <FileHandler.h>
#include <SettingsData.h>
#include <ConsoleContext.h>


//#define MEASURE_RBODIES

using namespace std;


const double App::DTCAP=0.5;

App::App(HINSTANCE p_hInstance, unsigned int p_width/*=1280*/, unsigned int p_height/*=1024*/)
{
	// ====================================
	// Simulation variable init
	// ====================================
	m_initWindowWidth = p_width;
	m_initWindowHeight = p_height;
	m_initWindowMode = true;

	m_fpsUpdateTick = 0.0f;
	m_controller = new TempController(-8.0f, 2.5f, 0.0f, 0.0f);
	m_controller->rotate(glm::vec3(0.0f, -HALFPI, 0.0f));
	m_context = NULL;
	m_graphicsDevice = NULL;
	m_input = NULL;
	m_toolBar = NULL;
	m_debugDrawBatch = NULL;
	m_debugDrawer = NULL;

	m_vp = NULL;
	m_timeScale = 1.0f;
	m_timeScaleToggle = false;
	m_timePauseStepToggle = false;
	m_time = 0.0;
	m_restart = false;

	m_consoleMode = false;
	m_measurementRuns = 1;
	//
	m_triggerPause = false;


	// ====================================
	// Member systems
	// ====================================
	m_renderSystem = NULL;
	m_rigidBodySystem = NULL;



	// ====================================
	// Environment init
	// ====================================
	// Context
	if (!m_consoleMode)
	{
		try
		{
			m_context = new Context(p_hInstance, "loppan",
				m_initWindowWidth, m_initWindowHeight);
		}
		catch (ContextException& e)
		{
			DEBUGWARNING((e.what()));
		}
		// Graphics
		try
		{
			m_graphicsDevice = new GraphicsDevice(m_context->getWindowHandle(),
				m_initWindowWidth, m_initWindowHeight, m_initWindowMode);
		}
		catch (GraphicsException& e)
		{
			DEBUGWARNING((e.what()));
		}
		m_toolBar = new Toolbar((void*)m_graphicsDevice->getDevicePointer());
		m_toolBar->setWindowSize(m_graphicsDevice->getWidth(), m_graphicsDevice->getHeight());
		m_context->addSubProcess(m_toolBar); // add toolbar to context (for catching input)
		m_debugDrawBatch = new DebugDrawBatch();
		m_debugDrawer = new DebugDrawer((void*)m_graphicsDevice->getDevicePointer(),
			(void*)m_graphicsDevice->getDeviceContextPointer(), m_debugDrawBatch);
		m_debugDrawer->setDrawArea((float)m_graphicsDevice->getWidth(), (float)m_graphicsDevice->getHeight());
		// input
		m_input = new Input();
		m_input->doStartup(m_context->getWindowHandle());
		// basic view orientation buffer
		m_vp = m_graphicsDevice->getBufferFactoryRef()->createMat4CBuffer();
	}
	else if (m_consoleMode)	// Console mode
	{
		ConsoleContext::init();
	}
	// ====================================

	// Global toolbar vars
	// ====================================
	if (m_toolBar)
	{
		m_toolBar->addReadOnlyVariable(Toolbar::PLAYER, "Real time", Toolbar::DOUBLE, &m_time);
		m_toolBar->addReadOnlyVariable(Toolbar::PLAYER, "Frame time", Toolbar::DOUBLE, &m_frameTime);
		m_toolBar->addReadWriteVariable(Toolbar::PLAYER, "Physics time scale", Toolbar::FLOAT, &m_timeScale);
		m_toolBar->addButton(Toolbar::PLAYER, "Play/Pause", boolButton, (void*)&m_triggerPause);
		m_toolBar->addButton(Toolbar::PLAYER, "Restart", boolButton, (void*)&m_restart);
	}
	// ====================================
}

App::~App()
{	
	// UNSUBSCRIPTIONS
	if (m_toolBar) m_context->removeSubProcessEntry(m_toolBar);
	// DELETES
	SAFE_DELETE(m_toolBar);
	SAFE_DELETE(m_debugDrawer);
	SAFE_DELETE(m_debugDrawBatch);
	SAFE_DELETE(m_graphicsDevice);
	SAFE_DELETE(m_context);
	SAFE_DELETE(m_input);
	SAFE_DELETE(m_controller);
	if (m_consoleMode)
	{
		ConsoleContext::end();
	}
	//
	//delete m_instances;
	SAFE_DELETE(m_vp);
}

void App::run()
{
	// Normal inits
	int fixedStepCounter = 0;
	bool dbgDrawAllChars = true;
	double controllerSystemTimingMs = 0.0;
	bool lockLFY_onRestart = false;
	if (m_toolBar)
	{
		if (m_debugDrawBatch) m_toolBar->addReadWriteVariable(Toolbar::PLAYER, "Enable DbgDraw", Toolbar::BOOL, &m_debugDrawBatch->m_enabled);
	}



	// ===========================================================
	// 
	//					  MAIN APP RESTART LOOP
	//
	// ===========================================================
#pragma region mainrestartloop
	do
	{
		m_restart = false;
		// Set up windows timer
		LARGE_INTEGER ticksPerSec = Time::getTicksPerSecond();
		double secondsPerTick = 1.0 / (double)ticksPerSec.QuadPart;
		// The physics clock is just used to run the physics and runs asynchronously with the gameclock
		LARGE_INTEGER currTimeStamp = Time::getTimeStamp();
		LARGE_INTEGER prevTimeStamp = currTimeStamp;
		// There's an inner loop in here where things happen once every TickMs. These variables are for that.
		LARGE_INTEGER gameClockTimeOffsetStamp = Time::getTimeStamp();
		double gameClockTimeOffset = (double)gameClockTimeOffsetStamp.QuadPart * secondsPerTick;
		const unsigned int gameTickMs = 16;

		double gameTickS = (double)gameTickMs / 1000.0;
		// Absolute start
		double timeStart = (double)Time::getTimeStamp().QuadPart * secondsPerTick;

		// Bullet physics initialization
		// Broadphase object
		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		// Collision dispatcher with default config
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
		// Register collision algorithm (needed for mesh collisions)
		// btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
		// Register physics solver
		// (Single threaded)
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
		// ==================================
		// Create the physics world
		// ==================================
		btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		dynamicsWorld->setGravity(btVector3(0, WORLD_GRAVITY, 0));

		// Measurements and debug
		MeasurementBin<string> rigidBodyStateDbgRecorder;

		// Artemis
		// Create and initialize systems
		artemis::SystemManager * sysManager = m_world.getSystemManager();
		if (m_toolBar) AdvancedEntitySystem::registerDebugToolbar(m_toolBar);
		if (m_debugDrawBatch) AdvancedEntitySystem::registerDebugDrawBatch(m_debugDrawBatch);
		//MovementSystem * movementsys = (MovementSystem*)sm->setSystem(new MovementSystem());
		//addGameLogic(movementsys);
#ifdef MEASURE_RBODIES
		m_rigidBodySystem = (RigidBodySystem*)sysManager->setSystem(new RigidBodySystem(dynamicsWorld, &rigidBodyStateDbgRecorder));
#else

		m_rigidBodySystem = (RigidBodySystem*)sysManager->setSystem(new RigidBodySystem(dynamicsWorld));
#endif
		//ConstantForceSystem* cforceSystem = (ConstantForceSystem*)sysManager->setSystem(new ConstantForceSystem());

		if (!m_consoleMode)
			m_renderSystem = (RenderSystem*)sysManager->setSystem(new RenderSystem(m_graphicsDevice));
		PositionRefSystem* posRefSystem = (PositionRefSystem*)sysManager->setSystem(new PositionRefSystem());


		ConstraintSystem* constraintSystem = (ConstraintSystem*)sysManager->setSystem(new ConstraintSystem(dynamicsWorld));
		sysManager->initializeAll();


		// Order independent
		addOrderIndependentSystem(constraintSystem);
		addOrderIndependentSystem(posRefSystem);

		// Combine Physics with our stuff!
		PhysicsWorldHandler physicsWorldHandler(dynamicsWorld/*, m_controllerSystem*/);
		//physicsWorldHandler.addOrderIndependentSystem(cforceSystem);
		physicsWorldHandler.addPreprocessSystem(m_rigidBodySystem);


		// Entity manager fetch
		artemis::EntityManager * entityManager = m_world.getEntityManager();

		// Create a ground entity
		artemis::Entity & ground = entityManager->create();
		ground.addComponent(new RigidBodyComponent(new btBoxShape(btVector3(400.0f, 10.0f, 400.0f)), 0.0f,
			CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT, CollisionLayer::COL_CHARACTER | CollisionLayer::COL_DEFAULT));
		ground.addComponent(new RenderComponent());
		MaterialComponent* mat = new MaterialComponent(dawnBringerPalRGB[COL_ASPHALT]);
		ground.addComponent(mat);
		ground.addComponent(new TransformComponent(glm::vec3(0.0f, -10.0f, 0.0f),
			glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)),
			glm::vec3(800.0f, 20.0f, 800.0f)));
		ground.refresh();


		// Test of controller
#ifdef CHARSIM
		float scale = 2.0f;
		float hipCoronalOffset = scale*0.2f; // coronal distance between hip joints and center
		glm::vec3 bodOffset;
		// size setups
		float	lfHeight	= scale*0.48f;
		float	uLegHeight	= scale*0.45f;
		float	lLegHeight	= scale*0.45f;
		float	footHeight	= scale*0.05f;
		float	footLen		= scale*0.3f;
		int chars = m_initCharCountSerial;
		bool lockPos = true;
		bool drawAll = dbgDrawAllChars;
		bool quadruped = false;
		
		if (m_characterCreateType == CharCreateType::QUADRUPED) 
		{
			quadruped = true;
		}
		else if (m_characterCreateType == CharCreateType::BIPED) 
		{	
			quadruped = false;
		}
		

		float charOffsetX = m_initCharOffset;
		float charPosY = lfHeight*0.5f + uLegHeight + lLegHeight + footHeight;

		DEBUGPRINT((ToString(charPosY).c_str()));

		float lfDist = 2.0f;
		int spineParts = 4;
		if (m_runOptimization)
		{
			charOffsetX = 0.0f;
			/*if (quadruped) chars = 5; else */chars = 10;
		}

		if (quadruped)
		{
			lLegHeight = scale*0.4f;
			footHeight = scale*0.05f;
			footLen = scale*0.2f;
			charPosY = lfHeight*0.5f + uLegHeight + lLegHeight + footHeight;
#pragma region quadrupedtype
			for (int x = 0; x < chars; x++) // number of characters
			{
				vector<artemis::Entity*> charLFs;
				vector<artemis::Entity*> hipJoints;

				std::vector<float> uLegLens; std::vector<float> lLegLens;
				std::vector<int> kneeFlip;

				int legFrames = 2;
				artemis::Entity* prevlegFrame = NULL;
				for (int y = 0; y < legFrames; y++) // number of leg frames
				{
					artemis::Entity& legFrame = entityManager->create();
					glm::vec3 pos = bodOffset + glm::vec3(/*x*charOffsetX*/0.0f, charPosY, (float)-y*lfDist);

					// if locked, we move down a tiny bit to get traction
					//if (lockLFY_onRestart) pos.y -= 0.5f*footHeight;

					//(float(i) - 50, 10.0f+float(i)*4.0f, float(i)*0.2f-50.0f);
					glm::vec3 lfSize = glm::vec3(hipCoronalOffset*2.0f, lfHeight, (float)(2 - y)*hipCoronalOffset);
					float characterMass = /*scale**/10.0f;
					RigidBodyComponent* lfRB = new RigidBodyComponent(new btBoxShape(btVector3(lfSize.x, lfSize.y, lfSize.z)*0.5f), characterMass,
						CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT);
					legFrame.addComponent(lfRB);
					if (drawAll || x == 0) legFrame.addComponent(new RenderComponent());
					MaterialComponent* matlf = new MaterialComponent(dawnBringerPalRGB[(x * 2) % 31]);
					legFrame.addComponent(matlf);
					TransformComponent* tc = new TransformComponent(pos,
						glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)),
						lfSize);
					tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
					legFrame.addComponent(tc);

					if (lockPos)
					{
						float lck = lockLFY_onRestart ? 0.0f : 1.0f;
						lfRB->setLinearFactor(glm::vec3(lck, 1, 1));
						lfRB->setAngularFactor(glm::vec3(1, lck, lck));
					}


					//legFrame.addComponent(new ConstantForceComponent(glm::vec3(0, characterMass*12.0f, 0)));
					legFrame.refresh();
					string legFrameName = "LegFrame";
					/*m_toolBar->addLabel(Toolbar::CHARACTER, legFrameName.c_str(), (" label='" + legFrameName + "'").c_str());*/
					if (x == 0 && m_toolBar) m_toolBar->addSeparator(Toolbar::CHARACTER, NULL, (" group='" + legFrameName + "'").c_str());
					//
					// Number of leg frames per character
					for (int n = 0; n < 2; n++) // number of legs per frame
					{
						string sideName = ToString(y) + (string(n == 0 ? "Left" : "Right") + "Leg");
						if (x == 0 && m_toolBar)
						{
							m_toolBar->addSeparator(Toolbar::CHARACTER, NULL, (" group='" + sideName + "' ").c_str());
							m_toolBar->defineBarParams(Toolbar::CHARACTER, ("/" + sideName + " opened=false").c_str());
						}
						//m_toolBar->addLabel(Toolbar::CHARACTER, sideName.c_str(),"");
						artemis::Entity* prev = &legFrame;
						artemis::Entity* upperLegSegment = NULL;
						float currentHipJointCoronalOffset = (float)(n * 2 - 1)*hipCoronalOffset;
						glm::vec3 legpos = pos + glm::vec3(currentHipJointCoronalOffset, 0.0f, 0.0f);
						glm::vec3 boxSize = glm::vec3(0.25f, uLegHeight, 0.25f);
						glm::vec3 parentSz = glm::vec3(boxSize.x, lfHeight, boxSize.z);
						for (int i = 0; i < 3; i++) // number of segments per leg
						{
							artemis::Entity & childJoint = entityManager->create();
							float jointXOffsetFromParent = 0.0f; // for coronal displacement for hip joints
							float jointYOffsetInChild = 0.0f; // for sagittal displacement for feet
							float jointZOffsetInChild = 0.0f; // for sagittal displacment for feet
							if (i != 0) parentSz = boxSize;//glm::vec3(boxSize.x, uLegHeight, boxSize.z);
							//boxSize = glm::vec3(0.25f, uLegHeight, 0.25f); // set new size for current box
							// segment specific constraint params
							glm::vec3 lowerAngleLim = glm::vec3(-HALFPI, -HALFPI*0.5f, -HALFPI*0.5f);
							glm::vec3 upperAngleLim = glm::vec3(HALFPI, HALFPI*0.5f, HALFPI*0.5f);
							string partName;
							float segmentMass = 5.0f;
							bool foot = false;
							float thisFootHeight = footHeight;
							float thisFootLen = footLen;
							/*if (y == 0) // digitigrade front feet
								thisFootHeight = footHeight * 4;*/
							if (i == 0) // if hip joint (upper leg)
							{
								partName = " upper";
								upperLegSegment = &childJoint;
								jointXOffsetFromParent = currentHipJointCoronalOffset;
								//lowerAngleLim = glm::vec3(-HALFPI, -HALFPI*0.5f, -HALFPI*0.0f);
								//upperAngleLim = glm::vec3(HALFPI, HALFPI*0.5f, HALFPI*0.0f);
								lowerAngleLim = glm::vec3(-HALFPI*0.2f, -HALFPI*0.5f*0.0f, -HALFPI*0.1f*0.0f);
								upperAngleLim = glm::vec3(HALFPI, HALFPI*0.5f*0.0f, HALFPI*0.1f*0.0f);
								segmentMass = /*scale**/5.0f;
								float height = uLegHeight;
								/*if (y == 0) // front legs
									height = height*0.8f;*/
								boxSize = glm::vec3(scale*0.1f, height, scale*0.1f);
								if (m_runOptimization)
								{
									if (n == 0) uLegLens.push_back(height);
								}
								//lowerAngleLim = glm::vec3(1, 1, 1);
								//upperAngleLim = glm::vec3(0,0,0);
							}
							else if (i == 1) // if knee (lower leg)
							{
								partName = " lower";
								if (y == 0) // front legs have "flipped" knees, for digitigrade anatomy
								{
									lowerAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
									upperAngleLim = glm::vec3(PI*0.5f, 0.0f, 0.0f);
								}
								else
								{
									lowerAngleLim = glm::vec3(-PI*0.7f/*-HALFPI*/, 0.0f, 0.0f);
									upperAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
								}
								segmentMass = /*scale**/4.0f;
								float kneeheight = lLegHeight;
								/*if (y == 0) // front legs
									height = height*0.8f;*/
								boxSize = glm::vec3(scale*0.1f, lLegHeight, scale*0.1f);
								if (m_runOptimization)
								{
									if (n == 0)
									{
										lLegLens.push_back(kneeheight + thisFootHeight);
										kneeFlip.push_back(y == 1 ? 1 : -1);
									}
								}
							}
							else if (i == 2) // if foot
							{
								partName = " foot";
								//boxSize = glm::vec3(scale*0.08f, footHeight, scale*0.2f);
								if (y == 0) // digitigrade front feet
								{
									lowerAngleLim = glm::vec3(HALFPI*0.3f, 0.0f, 0.0f);
									upperAngleLim = glm::vec3(HALFPI*1.01f, 0.0f, 0.0f);
									thisFootLen = 1.5f*footLen;
									//thisFootLen = footLen*0.5f;
								}
								else // digitigrade back feet
								{
									thisFootLen = 1.5f*footLen;
									lowerAngleLim = glm::vec3(HALFPI, 0.0f, 0.0f);
									upperAngleLim = glm::vec3(HALFPI*1.2f, 0.0f, 0.0f);
									//lowerAngleLim = glm::vec3(HALFPI*0.5f, -HALFPI*0.1f*0.0f, -HALFPI*0.1f);
									//upperAngleLim = glm::vec3(HALFPI*1.2f, HALFPI*0.1f*0.0f, HALFPI*0.1f);
								}
								jointYOffsetInChild = thisFootLen*0.2f;
								//jointYOffsetInChild = footLen*0.5f;
								jointZOffsetInChild = -thisFootHeight*0.5f;
								//lowerAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
								//upperAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
								//lowerAngleLim = glm::vec3(HALFPI*0.6f, -HALFPI*0.1f, -HALFPI*0.1f);
								//upperAngleLim = glm::vec3(HALFPI*1.8f, HALFPI*0.1f, HALFPI*0.1f);
								segmentMass = /*scale**/1.0f;
								foot = true;
								boxSize = glm::vec3(scale*0.2f, thisFootLen, thisFootHeight);
							}
							string dbgGrp = (" group='" + sideName + "'");
							if (x == 0 && m_toolBar) m_toolBar->addLabel(Toolbar::CHARACTER, (ToString(x) + sideName.substr(0, 2) + partName).c_str(), dbgGrp.c_str());
							legpos += glm::vec3(glm::vec3(0.0f, -parentSz.y*0.5f - boxSize.y*0.5f, 0.0f/*jointZOffsetInChild*/));
							if (foot == true)
							{
								// foot need collision callback properties
								childJoint.addComponent(new RigidBodyComponent(RigidBodyComponent::REGISTER_COLLISIONS,
									new btBoxShape(btVector3(boxSize.x, boxSize.y, boxSize.z)*0.5f), segmentMass, // note, h-lengths
									CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT));
							}
							else
							{
								// ordinary joint does not need collision callback
								childJoint.addComponent(new RigidBodyComponent(new btBoxShape(btVector3(boxSize.x, boxSize.y, boxSize.z)*0.5f), segmentMass, // note, h-lengths
									CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT));
							}
							if (drawAll || x == 0) childJoint.addComponent(new RenderComponent());
							if (i != 2)
							{			
								tc = new TransformComponent(legpos,
									glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)),
									boxSize);// note scale, so full lengths
								tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
								childJoint.addComponent(tc);
							}
							else // foot
							{
								// TODO! digitigrade feet
								//if (y == 0)// digitigrade front feet
								//{
								//	glm::quat rot = glm::quat(glm::vec3(-HALFPI, 0.0f, 0.0f));
								//	childJoint.addComponent(new TransformComponent(legpos + glm::vec3(0.0f, footLen*0.5f + jointZOffsetInChild, footLen*0.5f - jointYOffsetInChild),
								//		rot,
								//		boxSize));					// note scale, so full lengths
								//}
								//else// digitigrade back feet
								{
									glm::quat rot = glm::quat(glm::vec3(-HALFPI, 0.0f, 0.0f));
									// feet fly up for quadrupeds, optimize this for more correct: glm::vec3(0.0f, footLen*0.5f + jointZOffsetInChild, boxSize.y*0.5f - jointYOffsetInChild),

									tc = new TransformComponent(legpos + glm::vec3(0.0f, thisFootLen*0.5f + jointZOffsetInChild, thisFootLen*0.5f - jointYOffsetInChild),
										rot,
										boxSize);					// note scale, so full lengths
									tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
									childJoint.addComponent(tc);
								}
							}
							MaterialComponent* mat = new MaterialComponent(colarr[(y + n) * 3 + i]);
							childJoint.addComponent(mat);
							if (x == 0 && m_toolBar) m_toolBar->addReadWriteVariable(Toolbar::CHARACTER, (sideName.substr(0, 2) + ToString(partName[1]) + " Color").c_str(), Toolbar::COL_RGBA, (void*)&mat->getColorRGBA(), dbgGrp.c_str());
							ConstraintComponent::ConstraintDesc constraintDesc{ glm::vec3(0.0f, boxSize.y*0.5f - jointYOffsetInChild, -jointZOffsetInChild),	  // child (this)
								glm::vec3(jointXOffsetFromParent, -parentSz.y*0.5f, 0.0f),													  // parent
								{ lowerAngleLim, upperAngleLim },
								false };
							childJoint.addComponent(new ConstraintComponent(prev, constraintDesc));
							childJoint.refresh();
							prev = &childJoint;
						}
						hipJoints.push_back(upperLegSegment);
					}
					charLFs.push_back(&legFrame);
					prevlegFrame = &legFrame;
				} // leg frames
				//
				glm::vec3 pos = bodOffset + glm::vec3(/*x*charOffsetX*/0.0f, charPosY, -hipCoronalOffset*0.5f);
				artemis::Entity* prev = charLFs[0]; // the first parent is the first leg frame
				// The length of a spine (the height of its rotated segment) is the same as=
				// The distance between the LFs minus the h-length of a LFs, times two; divided 
				// by number of wanted spines:
				float boxHeight = (lfDist - (hipCoronalOffset*0.5f * 2)) / (float)spineParts;
				float spineHeight = lfHeight*0.75f;
				glm::vec3 boxSize = glm::vec3(hipCoronalOffset, boxHeight, spineHeight); // note, we rotate it
				glm::vec3 spinepos = pos + glm::vec3(0.0f, (lfHeight - spineHeight)*0.5f, -boxHeight*0.5f);
				// first spine joint is child to leg frame
				glm::vec3 parentSz = glm::vec3(boxSize.x, lfHeight, hipCoronalOffset);
				float jointYOffsetInParent = lfHeight*0.5f; // for sagittal displacement
				float jointZOffsetInParent = -parentSz.z*0.5f; // for sagittal displacment
				glm::vec3 lowerAngleLim = glm::vec3(-HALFPI*0.1f, -HALFPI*0.1f, -HALFPI*0.1f);
				glm::vec3 upperAngleLim = glm::vec3(HALFPI*0.1f, HALFPI*0.1f, HALFPI*0.1f);
				glm::vec3 lowerAngleLimBase = glm::vec3(-HALFPI, 0, 0);
				glm::vec3 upperAngleLimBase = glm::vec3(-HALFPI, 0, 0);
				std::vector<artemis::Entity*> spines;
				for (int s = 0; s < spineParts; s++)
				{
					// Create the spine
					// ----------------------------
					// SPINE
					// ----------------------------
					artemis::Entity & spineJoint = entityManager->create();
					if (s != 0) parentSz = boxSize;//glm::vec3(boxSize.x, uLegHeight, boxSize.z);
					float segmentMass = 1.0f;

					if (s > 0)
					{
						// as the non-root spines are children to the root spine, which is rotated,
						// we're working from another coordinate system for joint offsets and angle limits:
						spinepos += glm::vec3(glm::vec3(0.0f, 0.0f, -boxHeight));
						lowerAngleLimBase = glm::vec3(0.0f, 0, 0);
						upperAngleLimBase = glm::vec3(0.0f, 0, 0);
						jointYOffsetInParent = -parentSz.y*0.5f; // for sagittal displacement
						jointZOffsetInParent = -spineHeight*0.5f; // for sagittal displacment
					}

					// no need collision callback
					spineJoint.addComponent(new RigidBodyComponent(new btBoxShape(btVector3(boxSize.x, boxSize.y, boxSize.z)*0.5f), segmentMass, // note, h-lengths
						CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT));

					if (drawAll || x == 0) spineJoint.addComponent(new RenderComponent());


					TransformComponent* tc = new TransformComponent(spinepos,
						glm::quat(glm::vec3(HALFPI, 0.0f, 0.0f)),
						boxSize);
					tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
					spineJoint.addComponent(tc);

					MaterialComponent* mat = new MaterialComponent(colarr[s + 3]);
					spineJoint.addComponent(mat);

					ConstraintComponent::ConstraintDesc constraintDesc{ glm::vec3(0.0f, boxSize.y*0.5f, -spineHeight*0.5f),	  // child (this)
						glm::vec3(0.0f, jointYOffsetInParent, jointZOffsetInParent),							  // parent
						{ lowerAngleLimBase + lowerAngleLim, upperAngleLimBase + upperAngleLim },
						false };
					spineJoint.addComponent(new ConstraintComponent(prev, constraintDesc));
					spineJoint.refresh();
					spines.push_back(&spineJoint);
					prev = &spineJoint;
				}
				// finish by constraint the back LF to the last spine
				// the back LF become the child of the last spine joint
				jointYOffsetInParent = -boxSize.y*0.5f; // for sagittal displacement
				jointZOffsetInParent = -spineHeight*0.5f; // for sagittal displacment
				lowerAngleLim = glm::vec3(HALFPI, 0, 0);
				upperAngleLim = glm::vec3(HALFPI, 0, 0);
				ConstraintComponent::ConstraintDesc constraintDesc{ glm::vec3(0.0f, lfHeight*0.5f, hipCoronalOffset*0.5f),	  // child (this) ie. the LF
					glm::vec3(0.0f, jointYOffsetInParent, jointZOffsetInParent),							 // parent ie. the last spine joint
					{ lowerAngleLim, upperAngleLim },
					false };
				// parent to the last leg frame that was added
				prevlegFrame->addComponent(new ConstraintComponent(prev, constraintDesc));
				prevlegFrame->refresh();


				// ------------------

				// Controller
				artemis::Entity & controller = entityManager->create();
				ControllerComponent* controllerComp = new ControllerComponent(charLFs, hipJoints, &spines);
				controller.addComponent(controllerComp);
				if (m_runOptimization)
				{
					ControllerMovementRecorderComponent* recComp = new ControllerMovementRecorderComponent();
					recComp->setLowerLegLengths(lLegLens);
					recComp->setUpperLegLengths(uLegLens);
					// If first optimization run, and controller 0, add the current settings
					// as the "ghost" reference to measure for leg movements (measure physics result to kinematic target)
					// If this is another iteration, or another controller, add the reference ghost that we saved in the beginning.
					// The ghost is thus not changed between runs or controllers, everyone measures to the same reference.
					for (unsigned int r = 0; r < controllerComp->getLegFrameCount(); r++)
					{
						if (x == 0 && r >= (unsigned int)baseOptimizationReferenceMovementControllers.size())
						{
							baseOptimizationReferenceMovementControllers.push_back(ReferenceLegMovementController(controllerComp, controllerComp->getLegFrame(r), 2, kneeFlip[r]));
						}
						recComp->addLegReferenceController(baseOptimizationReferenceMovementControllers[r]);
					}
					controller.addComponent(recComp);

				}
				else if (m_bestParams!=NULL)// normal run and we have new param list loaded from file
				{
					controllerComp->setInitParams(*m_bestParams);
				}
				controller.refresh();
			}
#pragma endregion quadrupedtype
		}
		else
		{
#pragma region biped
			for (int x = 0; x < chars; x++) // number of characters
			{
				vector<artemis::Entity*> charLFs;
				vector<artemis::Entity*> hipJoints;

				std::vector<float> uLegLens; std::vector<float> lLegLens;

				for (int y = 0; y < 1; y++) // number of leg frames
				{
					artemis::Entity& legFrame = entityManager->create();
					glm::vec3 pos = bodOffset + glm::vec3(/*x*charOffsetX*/0.0f, charPosY, (float)-y);

					// if locked, we move down a tiny bit to get traction
					//if (lockLFY_onRestart) pos.y -= 0.5f*footHeight;

					//(float(i) - 50, 10.0f+float(i)*4.0f, float(i)*0.2f-50.0f);
					glm::vec3 lfSize = glm::vec3(hipCoronalOffset*2.0f, lfHeight, hipCoronalOffset);
					float characterMass = /*scale**/10.0f;
					RigidBodyComponent* lfRB = new RigidBodyComponent(new btBoxShape(btVector3(lfSize.x, lfSize.y, lfSize.z)*0.5f), characterMass,
						CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT);
					legFrame.addComponent(lfRB);
					if (drawAll || x == 0) legFrame.addComponent(new RenderComponent());
					MaterialComponent* matlf = new MaterialComponent(dawnBringerPalRGB[(x * 2) % 31]);
					legFrame.addComponent(matlf);
					TransformComponent* tc = new TransformComponent(pos,
						glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)),
						lfSize);
					tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
					legFrame.addComponent(tc);

					if (lockPos)
					{
						float lck = lockLFY_onRestart ? 0.0f : 1.0f;
						lfRB->setLinearFactor(glm::vec3(lck, 1, 1));
						lfRB->setAngularFactor(glm::vec3(1, lck, lck));
					}


					//legFrame.addComponent(new ConstantForceComponent(glm::vec3(0, characterMass*12.0f, 0)));
					legFrame.refresh();
					string legFrameName = "LegFrame";
					/*m_toolBar->addLabel(Toolbar::CHARACTER, legFrameName.c_str(), (" label='" + legFrameName + "'").c_str());*/
					if (x == 0 && m_toolBar) m_toolBar->addSeparator(Toolbar::CHARACTER, NULL, (" group='" + legFrameName + "'").c_str());
					//
					// Number of leg frames per character
					for (int n = 0; n < 2; n++) // number of legs per frame
					{
						string sideName = (string(n == 0 ? "Left" : "Right") + "Leg");
						if (x == 0 && m_toolBar)
						{
							m_toolBar->addSeparator(Toolbar::CHARACTER, NULL, (" group='" + sideName + "' ").c_str());
							m_toolBar->defineBarParams(Toolbar::CHARACTER, ("/" + sideName + " opened=false").c_str());
						}
						//m_toolBar->addLabel(Toolbar::CHARACTER, sideName.c_str(),"");
						artemis::Entity* prev = &legFrame;
						artemis::Entity* upperLegSegment = NULL;
						float currentHipJointCoronalOffset = (float)(n * 2 - 1)*hipCoronalOffset;
						glm::vec3 legpos = pos + glm::vec3(currentHipJointCoronalOffset, 0.0f, 0.0f);
						glm::vec3 boxSize = glm::vec3(0.25f, uLegHeight, 0.25f);
						glm::vec3 parentSz = glm::vec3(boxSize.x, lfHeight, boxSize.z);
						for (int i = 0; i < 3; i++) // number of segments per leg
						{
							artemis::Entity & childJoint = entityManager->create();
							float jointXOffsetFromParent = 0.0f; // for coronal displacement for hip joints
							float jointYOffsetInChild = 0.0f; // for sagittal displacement for feet
							float jointZOffsetInChild = 0.0f; // for sagittal displacment for feet
							if (i != 0) parentSz = boxSize;//glm::vec3(boxSize.x, uLegHeight, boxSize.z);
							//boxSize = glm::vec3(0.25f, uLegHeight, 0.25f); // set new size for current box
							// segment specific constraint params
							glm::vec3 lowerAngleLim = glm::vec3(-HALFPI, -HALFPI*0.5f, -HALFPI*0.5f);
							glm::vec3 upperAngleLim = glm::vec3(HALFPI, HALFPI*0.5f, HALFPI*0.5f);
							string partName;
							float segmentMass = 5.0f;
							bool foot = false;
							if (i == 0) // if hip joint (upper leg)
							{
								partName = " upper";
								upperLegSegment = &childJoint;
								jointXOffsetFromParent = currentHipJointCoronalOffset;
								//lowerAngleLim = glm::vec3(-HALFPI, -HALFPI*0.5f, -HALFPI*0.0f);
								//upperAngleLim = glm::vec3(HALFPI, HALFPI*0.5f, HALFPI*0.0f);
								lowerAngleLim = glm::vec3(-HALFPI, -HALFPI*0.5f*0.0f, -HALFPI*0.1f*0.0f);
								upperAngleLim = glm::vec3(HALFPI, HALFPI*0.5f*0.0f, HALFPI*0.1f*0.0f);
								segmentMass = /*scale**/5.0f;
								boxSize = glm::vec3(scale*0.1f, uLegHeight, scale*0.1f);
								if (m_runOptimization)
								{
									if (n == 0) uLegLens.push_back(uLegHeight);
								}
								//lowerAngleLim = glm::vec3(1, 1, 1);
								//upperAngleLim = glm::vec3(0,0,0);
							}
							else if (i == 1) // if knee (lower leg)
							{
								partName = " lower";
								lowerAngleLim = glm::vec3(-PI*0.7f/*-HALFPI*/, 0.0f, 0.0f);
								upperAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
								segmentMass = /*scale**/4.0f;
								boxSize = glm::vec3(scale*0.1f, lLegHeight, scale*0.1f);
								if (m_runOptimization)
								{
									if (n == 0) lLegLens.push_back(lLegHeight + footHeight);
								}
							}
							else if (i == 2) // if foot
							{
								partName = " foot";
								//boxSize = glm::vec3(scale*0.08f, footHeight, scale*0.2f);
								boxSize = glm::vec3(scale*0.2f, footLen, footHeight);
								jointYOffsetInChild = footLen*0.2f;
								jointZOffsetInChild = 0.0f*-footHeight*0.5f;
								//jointZOffsetInChild = (boxSize.z - parentSz.z)*0.5f;
								lowerAngleLim = glm::vec3(HALFPI*0.8f, -HALFPI*0.1f*0.0f, -HALFPI*0.1f);
								upperAngleLim = glm::vec3(HALFPI*1.2f, HALFPI*0.1f*0.0f, HALFPI*0.1f);
								//lowerAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
								//upperAngleLim = glm::vec3(0.0f, 0.0f, 0.0f);
								//lowerAngleLim = glm::vec3(HALFPI*0.6f, -HALFPI*0.1f, -HALFPI*0.1f);
								//upperAngleLim = glm::vec3(HALFPI*1.8f, HALFPI*0.1f, HALFPI*0.1f);
								segmentMass = /*scale**/1.0f;
								foot = true;
							}
							string dbgGrp = (" group='" + sideName + "'");
							if (x == 0 && m_toolBar) m_toolBar->addLabel(Toolbar::CHARACTER, (ToString(x) + sideName[0] + partName).c_str(), dbgGrp.c_str());
							legpos += glm::vec3(glm::vec3(0.0f, -parentSz.y*0.5f - boxSize.y*0.5f, 0.0f/*jointZOffsetInChild*/));
							if (foot == true)
							{
								// foot need collision callback properties
								childJoint.addComponent(new RigidBodyComponent(RigidBodyComponent::REGISTER_COLLISIONS,
									new btBoxShape(btVector3(boxSize.x, boxSize.y, boxSize.z)*0.5f), segmentMass, // note, h-lengths
									CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT));
							}
							else
							{
								// ordinary joint does not need collision callback
								childJoint.addComponent(new RigidBodyComponent(new btBoxShape(btVector3(boxSize.x, boxSize.y, boxSize.z)*0.5f), segmentMass, // note, h-lengths
									CollisionLayer::COL_CHARACTER, CollisionLayer::COL_GROUND | CollisionLayer::COL_DEFAULT));
							}
							if (drawAll || x == 0) childJoint.addComponent(new RenderComponent());
							if (i != 2)
							{
								tc = new TransformComponent(legpos,
									glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)),
									boxSize);// note scale, so full lengths
								tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
								childJoint.addComponent(tc);
							}
							else // foot
							{
								glm::quat rot = glm::quat(glm::vec3(-HALFPI, 0.0f, 0.0f));
								tc = new TransformComponent(legpos + glm::vec3(0.0f, footLen*0.5f + jointZOffsetInChild, footLen*0.5f - jointYOffsetInChild),
									rot,
									boxSize);					// note scale, so full lengths
								tc->setPositionOffset(glm::vec3(x*charOffsetX, 0.0f, 0.0f));
								childJoint.addComponent(tc);
							}
							MaterialComponent* mat = new MaterialComponent(colarr[n * 3 + i]);
							childJoint.addComponent(mat);
							if (x == 0 && m_toolBar) m_toolBar->addReadWriteVariable(Toolbar::CHARACTER, (ToString(x) + sideName[1] + ToString(partName[1]) + " Color").c_str(), Toolbar::COL_RGBA, (void*)&mat->getColorRGBA(), dbgGrp.c_str());
							ConstraintComponent::ConstraintDesc constraintDesc{ glm::vec3(0.0f, boxSize.y*0.5f - jointYOffsetInChild, -jointZOffsetInChild),	  // child (this)
								glm::vec3(jointXOffsetFromParent, -parentSz.y*0.5f, 0.0f),													  // parent
								{ lowerAngleLim, upperAngleLim },
								false };
							childJoint.addComponent(new ConstraintComponent(prev, constraintDesc));
							childJoint.refresh();
							prev = &childJoint;
						}
						hipJoints.push_back(upperLegSegment);
					}
					charLFs.push_back(&legFrame);
				} // leg frames
				// Controller
				artemis::Entity & controller = entityManager->create();
				ControllerComponent* controllerComp = new ControllerComponent(charLFs, hipJoints);
				controller.addComponent(controllerComp);
				if (m_runOptimization)
				{
					ControllerMovementRecorderComponent* recComp = new ControllerMovementRecorderComponent();
					recComp->setLowerLegLengths(lLegLens);
					recComp->setUpperLegLengths(uLegLens);
					// If first optimization run, and controller 0, add the current settings
					// as the "ghost" reference to measure for leg movements (measure physics result to kinematic target)
					// If this is another iteration, or another controller, add the reference ghost that we saved in the beginning.
					// The ghost is thus not changed between runs or controllers, everyone measures to the same reference.
					for (int r = 0; r < controllerComp->getLegFrameCount(); r++)
					{
						if (x == 0 && r >= baseOptimizationReferenceMovementControllers.size())
						{
							baseOptimizationReferenceMovementControllers.push_back(ReferenceLegMovementController(controllerComp, controllerComp->getLegFrame(r)));
						}
						recComp->addLegReferenceController(baseOptimizationReferenceMovementControllers[r]);
					}
					controller.addComponent(recComp);

				}
				else if (m_bestParams != NULL)// normal run and we have new param list loaded from file
				{
					controllerComp->setInitParams(*m_bestParams);
				}
				controller.refresh();
			}

#pragma endregion biped
		}
		if (m_runOptimization)
		{
			m_optimizationSystem->initSim(bestOptimizationScore, m_bestParams);
		}
#endif

#ifdef MEASURE_RBODIES
		rigidBodyStateDbgRecorder.activate();
#endif


		// debug boxes
		for (int x = 0; x < 10; x++)
		for (int y = 0; y < 10; y++)
		for (int z = 0; z < 10; z++)
		{
			artemis::Entity & boxx = entityManager->create();
			glm::vec3 pos((x-5.0f)*2.0f,20.0f+(y-5.0f)*2.0f,(z-5.0f)*2.0f);
			glm::vec3 bfSize = glm::vec3(1.0f, 1.0f, 1.0f);
			RigidBodyComponent* btrb = new RigidBodyComponent(new btBoxShape(btVector3(bfSize.x, bfSize.y, bfSize.z)*0.5f), 10.0f,
				CollisionLayer::COL_DEFAULT, CollisionLayer::COL_DEFAULT | CollisionLayer::COL_CHARACTER);
			boxx.addComponent(btrb);
			boxx.addComponent(new RenderComponent());
			MaterialComponent* matbx = new MaterialComponent(colarr[x % colarrSz]);
			boxx.addComponent(matbx);
			boxx.addComponent(new TransformComponent(pos));
			//boxx.addComponent(new TransformComponent(pos,
			//	glm::inverse(glm::quat(m_controller->getRotationMatrix())),
			//	bfSize));
			//boxx.addComponent(new ConstantForceComponent(MathHelp::transformDirection(glm::inverse(m_controller->getRotationMatrix()), glm::vec3(0, 0, 300.0f)), 1.0f));
			boxx.refresh();
		}


		// Message pump struct
		MSG msg = { 0 };

		// secondary run variable
		// lets non-context systems quit the program
		bool run = true;

		//double fixedStep = 1.0 / 60.0;
		double physicsStep = 1.0 / 60.0;

		// Dry run, so artemis have run before physics first step
		gameUpdate(0.0f);
		//dynamicsWorld->stepSimulation((btScalar)fixedStep, 1, (btScalar)fixedStep);
		unsigned int oldSteps = physicsWorldHandler.getNumberOfInternalSteps();
		m_time = 0.0;
		//bool shooting = false;
#ifdef OPTIMSIM
		double optimizationDbgMaxscoreelem = 1.0f, 
			optimizationDbgBparamsmaxelem = 1.0f, 
			optimizationDbgBparamsminelem = 0.0f;
		if (m_runOptimization)
		{			
			if (allOptimizationResults.size() > 1) optimizationDbgMaxscoreelem = *std::max_element(allOptimizationResults.begin(), allOptimizationResults.end());
			if (m_bestParams != NULL && m_bestParams->size() > 1)
			{
				optimizationDbgBparamsmaxelem = *std::max_element(m_bestParams->begin(), m_bestParams->end());
				optimizationDbgBparamsminelem = *std::min_element(m_bestParams->begin(), m_bestParams->end());
			}
		}
#endif

		// ===========================================================
		// 
		//
		//					    MAIN GAME LOOP
		//
		//
		// ===========================================================
#pragma region mainloop
		while ((m_consoleMode || !m_context->closeRequested()) && run && !m_restart)
		{
			double startFrameTimeMs = Time::getTimeSeconds()*1000.0;
			if (!pumpMessage(msg))
			{				

				drawDebugAxes();
				
				// ====================================
				//			   Render 3D
				// ====================================
				render();


				m_time = (double)Time::getTimeStamp().QuadPart*secondsPerTick - timeStart;

				// ====================================
				//		   Physics update step
				// ====================================
				/* This, like the rendering, ticks every time around.
				Bullet does the interpolation for us. */
				currTimeStamp = Time::getTimeStamp();
				double phys_dt = (double)m_timeScale*(double)(currTimeStamp.QuadPart - prevTimeStamp.QuadPart) * secondsPerTick;

				// Tick the bullet world. Keep in mind that bullet takes seconds
				// timeStep < maxSubSteps * fixedTimeStep
				dynamicsWorld->stepSimulation((btScalar)phys_dt/*, 10*/,  1, (btScalar)physicsStep);
				// ========================================================

				unsigned int steps = physicsWorldHandler.getNumberOfInternalSteps();

				prevTimeStamp = currTimeStamp;

	

				// Game Clock part of the loop
				// ========================================================
				double dt=0.0;
				dt = ((double)Time::getTimeStamp().QuadPart*secondsPerTick - gameClockTimeOffset);

				// Game clock based updates
				while (dt >= gameTickS)
				{
					dt -= gameTickS;
					gameClockTimeOffset += gameTickS;
					// Handle all input
					processInput();
					// Update logic
					double interval = gameTickS;


					// shoot (temp code)
					//if (m_input)
					//{
					//	if (m_input->g_kb->isKeyDown(KC_X))
					//	{
					//		if (!shooting)
					//		{
					//			shooting = true;
					//			artemis::Entity & proj = entityManager->create();
					//			glm::vec3 pos = MathHelp::toVec3(m_controller->getPos());
					//			glm::vec3 bfSize = glm::vec3(1.0f, 1.0f, 1.0f);
					//			RigidBodyComponent* btrb = new RigidBodyComponent(new btBoxShape(btVector3(bfSize.x, bfSize.y, bfSize.z)*0.5f), 10.0f,
					//				CollisionLayer::COL_DEFAULT, CollisionLayer::COL_DEFAULT | CollisionLayer::COL_CHARACTER);
					//			proj.addComponent(btrb);
					//			proj.addComponent(new RenderComponent());
					//			MaterialComponent* matbx = new MaterialComponent(colarr[((int)m_time)%colarrSz]);
					//			proj.addComponent(matbx);
					//			proj.addComponent(new TransformComponent(pos,
					//				glm::inverse(glm::quat(m_controller->getRotationMatrix())),
					//				bfSize));
					//			proj.addComponent(new ConstantForceComponent(MathHelp::transformDirection(glm::inverse(m_controller->getRotationMatrix()), glm::vec3(0, 0, 300.0f)), 1.0f));
					//			proj.refresh();
					//		}
					//	}
					//	else
					//		shooting = false;
					//}

					handleContext(interval, phys_dt, steps - oldSteps);
					gameUpdate(interval);
				}

				// ========================================================
				oldSteps = physicsWorldHandler.getNumberOfInternalSteps();
				//
			}


			m_frameTime = Time::getTimeSeconds()*1000.0 - startFrameTimeMs;

		} // endwhile mainloop
#pragma endregion mainloop

		if (!m_restart)
			DEBUGPRINT(("\n\nSTOPPING APPLICATION\n\n"));



		// Clean up
		// artemis
		constraintSystem->removeAllConstraints();
		entityManager->removeAllEntities();
		m_orderIndependentSystems.clear();
		m_world.getSystemManager()->getSystems().deleteData();
		m_rigidBodySystem=NULL;
		m_renderSystem=NULL;

		// bullet
		//cleanup in the reverse order of creation/initialization

		//remove the rigidbodies from the dynamics world and delete them
		for (int bi = dynamicsWorld->getNumCollisionObjects() - 1; bi >= 0; bi--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[bi];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
				delete body->getMotionState();
			dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}


		delete broadphase;
		delete collisionConfiguration;
		delete dispatcher;
		delete solver;
		delete dynamicsWorld;

	} while (m_restart);
#pragma endregion mainrestartloop
}




void App::updateController(float p_dt)
{
	//m_controller->moveThrust(glm::vec3(0.0f, 0.0f, -0.8f));
	//m_controller->moveAngularThrust(glm::vec3(0.0f, 0.0f, 1.0f)*0.07f);
	// get joystate
	//Just dump the current joy state
	JoyStick* joy = nullptr;
	if (m_input->hasJoysticks()) 
		joy = m_input->g_joys[0];
	float thrustPow = 1.0f;
	// Thrust
	if (m_input->g_kb->isKeyDown(KC_LEFT) || m_input->g_kb->isKeyDown(KC_A))
		m_controller->moveThrust(glm::vec3(-1.0f,0.0f,0.0f)*thrustPow);
	if (m_input->g_kb->isKeyDown(KC_RIGHT) || m_input->g_kb->isKeyDown(KC_D))
		m_controller->moveThrust(glm::vec3(1.0f,0.0f,0.0f)*thrustPow);
	if (m_input->g_kb->isKeyDown(KC_UP) || m_input->g_kb->isKeyDown(KC_W))
		m_controller->moveThrust(glm::vec3(0.0f,1.0f,0.0f)*thrustPow);
	if (m_input->g_kb->isKeyDown(KC_DOWN) || m_input->g_kb->isKeyDown(KC_S))
		m_controller->moveThrust(glm::vec3(0.0f,-1.0f,0.0f)*thrustPow);
	if (m_input->g_kb->isKeyDown(KC_SPACE))
		m_controller->moveThrust(glm::vec3(0.0f,0.0f,1.0f)*thrustPow);
	if (m_input->g_kb->isKeyDown(KC_B))
		m_controller->moveThrust(glm::vec3(0.0f,0.0f,-1.0f)*thrustPow);
	// Joy thrust
	if (joy!=nullptr)
	{
		const JoyStickState& js = joy->getJoyStickState();
		m_controller->moveThrust(glm::vec3((float)(invclampcap(js.mAxes[1].abs,-5000,5000))* 0.0001f,
			(float)(invclampcap(js.mAxes[0].abs,-5000,5000))*-0.0001f,
			(float)(js.mAxes[4].abs)*-0.0001f)*thrustPow);
	}
	
	
	// Angular thrust
	if (m_input->g_kb->isKeyDown(KC_Q) || (joy!=nullptr && joy->getJoyStickState().mButtons[4]))
		m_controller->moveAngularThrust(glm::vec3(0.0f,0.0f,-1.0f));
	if (m_input->g_kb->isKeyDown(KC_E) || (joy!=nullptr && joy->getJoyStickState().mButtons[5]))
		m_controller->moveAngularThrust(glm::vec3(0.0f,0.0f,1.0f));
	if (m_input->g_kb->isKeyDown(KC_T))
		m_controller->moveAngularThrust(glm::vec3(0.0f,1.0f,0.0f));
	if (m_input->g_kb->isKeyDown(KC_R))
		m_controller->moveAngularThrust(glm::vec3(0.0f,-1.0f,0.0f));
	if (m_input->g_kb->isKeyDown(KC_U))
		m_controller->moveAngularThrust(glm::vec3(1.0f,0.0f,0.0f));
	if (m_input->g_kb->isKeyDown(KC_J))
		m_controller->moveAngularThrust(glm::vec3(-1.0f,0.0f,0.0f));
	// Joy angular thrust
	if (joy!=nullptr)
	{
		const JoyStickState& js = joy->getJoyStickState();
		m_controller->moveAngularThrust(glm::vec3((float)(invclampcap(js.mAxes[2].abs,-5000,5000))*-0.00001f,
			(float)(invclampcap(js.mAxes[3].abs,-5000,5000))*-0.00001f,
			0.0f));
	}
	
	float mousemovemultiplier=0.002f;
	float mouseX=(float)m_input->g_m->getMouseState().X.rel*mousemovemultiplier;
	float mouseY=(float)m_input->g_m->getMouseState().Y.rel*mousemovemultiplier;
	if ((abs(mouseX)>0.0f || abs(mouseY)>0.0f) && m_input->g_m->getMouseState().buttonDown(MB_Middle))
	{
		m_controller->rotate(glm::vec3(clamp(mouseY,-1.0f,1.0f),clamp(mouseX,-1.0f,1.0f),0.0f));
	}
}

bool App::pumpMessage( MSG& p_msg )
{
	bool res = PeekMessage(&p_msg, NULL, 0, 0, PM_REMOVE)>0?true:false;
	if (res)
	{
		TranslateMessage(&p_msg);
		DispatchMessage(&p_msg);
	}
	return res;
}


void App::processInput()
{
	if (m_input) m_input->run();
}

void App::handleContext(double p_dt, double p_physDt, unsigned int p_physSteps)
{
	// apply resizing on graphics device if it has been triggered by the context
	if (!m_consoleMode && m_context->isSizeDirty())
	{
		if (m_toolBar) m_toolBar->setWindowSize(0, 0);
		pair<int, int> sz = m_context->getSize();
		int width = sz.first, height = sz.second;
		m_graphicsDevice->updateResolution(width,height);
		if (m_toolBar) m_toolBar->setWindowSize(width, height);
		if (m_debugDrawer) m_debugDrawer->setDrawArea((float)width, (float)height);
	}
	// special console tick
	if (m_consoleMode) ConsoleContext::refreshConsole(p_dt);
	// Print fps in window head border
	m_fpsUpdateTick -= (float)p_dt;
	if (m_fpsUpdateTick <= 0.0f)
	{
		float fps = (1.0f / (float)(p_dt*1000.0f))*1000.0f;
		float pfps = 1.0f / (float)p_physDt;
		std::string title = " | Game FPS: " + ToString(fps) + " | Phys steps/frame: " + ToString(p_physSteps) + " | Phys FPS: " + ToString(pfps);
		if (!m_consoleMode)
		{
			m_context->updateTitle(title.c_str());
		}
		else
		{
			ConsoleContext::setTitle("DBGCON" + title);
		}
		m_fpsUpdateTick = 0.3f;
	}
}

void App::gameUpdate( double p_dt )
{
	float dt = (float)p_dt;
	float game_dt = m_timeScale*(float)p_dt;


	// temp controller update code
	if (m_input)
	{
		updateController(dt);
		m_controller->setFovFromAngle(52.0f, m_graphicsDevice->getAspectRatio());
		m_controller->update(dt);
	}

	// Get camera info to buffer
	if (m_vp)
	{
		std::memcpy(&m_vp->accessBuffer, &m_controller->getViewProjMatrix(), sizeof(float) * 4 * 4);
		m_vp->update();
	}

	if (m_timePauseStepToggle && m_timeScale > 0.0f)
		m_timeScale = 0.0f;
	if (m_input)
	{
		if (m_input->g_kb->isKeyDown(KC_RETURN) || m_triggerPause)
		{
			if (!m_timeScaleToggle)
			{
				if (m_timeScale < 1.0f)
					m_timeScale = 1.0f;
				else
					m_timeScale = 0.0f;
				m_timeScaleToggle = true;
				m_triggerPause = false;
			}
		}
		else
		{
			m_timeScaleToggle = false;
		}
		if (m_input->g_kb->isKeyDown(KC_NUMPAD6))
		{
			if (m_timeScale == 0.0f && !m_timePauseStepToggle)
			{
				m_timePauseStepToggle = true;
				m_timeScale = 1.0f;
			}
		}
		else
		{
			m_timePauseStepToggle = false;
		}
	}
	// If triggered from elsewhere
	/*if (m_timeScaleToggle && m_timeScale != 0.0f)
		m_timeScale = 0.0f;
	if (!m_timeScaleToggle && m_timeScale == 0.0f)
		m_timeScale = 1.0f;*/
	

	// Update entity systems
	m_world.loopStart();
	m_world.setDelta(game_dt);
	// Physics result gathering have to run first
	m_rigidBodySystem->executeDeferredConstraintInits();
	m_rigidBodySystem->process();
	m_rigidBodySystem->lateUpdate();
	// // Run all other systems, for which order doesn't matter
	processSystemCollection(&m_orderIndependentSystems);
	// // Render system is processed last
	if (m_renderSystem!=NULL) m_renderSystem->process();
}


void App::render()
{
	if (m_renderSystem!=NULL)
	{
		// Clear render targets
		m_graphicsDevice->clearRenderTargets();
		// Run passes
		if (m_vp) m_graphicsDevice->executeRenderPass(GraphicsDevice::P_BASEPASS, m_vp, m_renderSystem->getCulledInstanceBuffers(), NULL);
		m_graphicsDevice->executeRenderPass(GraphicsDevice::P_COMPOSEPASS);
		//if (m_vp) m_graphicsDevice->executeRenderPass(GraphicsDevice::P_BOUNDINGBOX_WIREFRAMEPASS, m_vp, m_renderSystem->getCulledInstanceBuffers());
		// Debug
		if (m_debugDrawer) m_debugDrawer->render(m_controller);
		if (m_toolBar) m_toolBar->draw();
		// Flip!
		m_graphicsDevice->flipBackBuffer();
		//
		// Clear debug draw batch 
		// (not optimal to only do it here if drawing from game systems,
		// batch calls should be put in a map or equivalent)
		//m_debugDrawBatch->clearDrawCalls();
	}
}

// Add a system for game logic processing
void App::addOrderIndependentSystem(artemis::EntityProcessingSystem* p_system)
{
	if (p_system!=NULL) m_orderIndependentSystems.push_back(p_system);
}


void App::processSystemCollection(vector<artemis::EntityProcessingSystem*>* p_systemCollection)
{
	unsigned int count = (unsigned int)p_systemCollection->size();
	for (unsigned int i = 0; i < count; i++)
	{
		artemis::EntityProcessingSystem* system = (*p_systemCollection)[i];
		system->process();
	}
}


void App::drawDebugAxes()
{
	// draw axes
	if (m_debugDrawBatch)
	{
		m_debugDrawBatch->drawLine(glm::vec3(0.0f), glm::vec3(10.0f, 0.0f, 0.0f), colarr[0], colarr[1]);
		m_debugDrawBatch->drawLine(glm::vec3(0.0f), glm::vec3(0.0f, 10.0f, 0.0f), colarr[3], colarr[4]);
		m_debugDrawBatch->drawLine(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 10.0f), dawnBringerPalRGB[COL_NAVALBLUE], dawnBringerPalRGB[COL_LIGHTBLUE]);
	}
}
