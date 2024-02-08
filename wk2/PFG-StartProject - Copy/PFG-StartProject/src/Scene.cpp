#include "Scene.h"


/*! \brief Brief description.
*  Scene class is a container for loading all the game objects in your simulation or your game.
*
*/
Scene::Scene()
{
	// Set up your scene here......
	// Set a camera
	_camera = new Camera();
	// Don't start simulation yet
	_simulation_start = false;

	// Position of the light, in world-space
	_lightPosition = glm::vec3(10, 10, 0);

	// Create a game object
	_physics_object_1 = new GameObject();
	_physics_object_2 = new GameObject();
	// Create a game level object
	_level = new GameObject();

	//Set initial velocity
	_v_i_1 = glm::vec3(0.0f, 0.0f, 0.0f);
	_v_i_2 = glm::vec3(0.0f, 0.0f, 0.0f);

	// Create the material for the game object- level
	Material *modelMaterial = new Material();
	// Shaders are now in files
	modelMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	modelMaterial->SetDiffuseColour(glm::vec3(0.8, 0.8, 0.8));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	modelMaterial->SetTexture("assets/textures/diffuse.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	modelMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_level->SetMaterial(modelMaterial);

	// The mesh is the geometry for the object
	Mesh *groundMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	groundMesh->LoadOBJ("assets/models/woodfloor.obj");
	// Tell the game object to use this mesh
	_level->SetMesh(groundMesh);
	_level->SetPosition(0.0f, 0.0f, 0.0f);
	_level->SetRotation(3.141590f, 0.0f, 0.0f);


	// Create the material for the game object- level
	Material *objectMaterial = new Material();
	// Shaders are now in files
	objectMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	objectMaterial->SetDiffuseColour(glm::vec3(0.8, 0.1, 0.1));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	objectMaterial->SetTexture("assets/textures/default.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	objectMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_physics_object_1->SetMaterial(objectMaterial);

	// Create the material for the game object- level 2
	Material* objectMaterial2 = new Material();
	// Shaders are now in files
	objectMaterial2->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	objectMaterial2->SetDiffuseColour(glm::vec3(0.8, 0.1, 0.1));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	objectMaterial2->SetTexture("assets/textures/default.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	objectMaterial2->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_physics_object_2->SetMaterial(objectMaterial2);

	// Set the geometry for the object
	Mesh *modelMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	modelMesh->LoadOBJ("assets/models/sphere.obj");
	// Tell the game object to use this mesh
	_physics_object_1->SetMesh(modelMesh);
	_physics_object_1->SetPosition(0.0f, 5.0f, 0.0f);
	_physics_object_1->SetScale(0.3f, 0.3f, 0.3f);

	_physics_object_2->SetMesh(modelMesh);
	_physics_object_2->SetPosition(5.0f, 5.0f, 0.0f);
	_physics_object_2->SetScale(0.3f, 0.3f, 0.3f);

	
}

Scene::~Scene()
{
	// You should neatly clean everything up here
	delete _physics_object_1;
	delete _physics_object_2;
	delete _level;
	delete _camera;
}

void Scene::Update(float deltaTs, Input* input)
{
	// Update the game object (this is currently hard-coded motion)
	if (input->cmd_x)
	{
		_simulation_start = true;
	}
	if (_simulation_start == true)
	{
		//glm::vec3 pos = _physics_object->GetPosition();
		//pos += glm::vec3(0.0, -deltaTs, 0.0);
		//_physics_object->SetPosition(pos);


		// Lab2: Use kinematics equations to compute kinematics motion
		// We compute the motion of the object with a series of time steps. 
		// For each of the time steps, following procedures should be carried out
		//     1. Get the current position of the object
		//     2. Given the initial quantities, Compute the velocity of the current time step 
		//     3. Use the kinematics equations to compute the x, y, and x position at the current time step
		//     4. Assign the current velocity to the initial velocity for the next time step 
		//     5. Set the current position for the object
		//     6  Do collision detections and responses
		//     7 Update the model matrix for graphics engine to draw 
		
		//number 1
		
		glm::vec3 pos1 = _physics_object_1->GetPosition();
		glm::vec3 vel1;
		float g = -9.8;
		pos1.x += _v_i_1.x * deltaTs;
		pos1.z += _v_i_1.z * deltaTs;
		vel1.x = _v_i_1.x;
		vel1.z = _v_i_1.z;
		vel1.y = _v_i_1.y + (g) * deltaTs;
		pos1.y += (_v_i_1.y + vel1.y) * 0.5f * deltaTs;
		_v_i_1 = vel1;

		// Lab 2: Exercise 4: compute collision with the ground

		if (pos1.y <= 0.3f) {
			pos1.y = 0.3f;
		}
		_physics_object_1->SetPosition(pos1);

		//number 2

		glm::vec3 pos2 = _physics_object_2->GetPosition();
		glm::vec3 vel2;
		pos2.x += _v_i_1.x * deltaTs;
		pos2.z += _v_i_1.z * deltaTs;
		vel2.x = _v_i_1.x;
		vel2.z = _v_i_1.z;
		vel2.y = _v_i_1.y + (g)*deltaTs;
		pos2.y += (_v_i_1.y + vel2.y) * 0.5f * deltaTs;
		_v_i_2 = vel2;

		// Lab 2: Exercise 4: compute collision with the ground

		if (pos2.y <= 0.3f) {
			pos2.y = 0.3f;
		}
		_physics_object_2->SetPosition(pos2);

	}
	_physics_object_1->Update(deltaTs);
	_physics_object_2->Update(deltaTs);
	_level->Update(deltaTs);
	_camera->Update(input);

	_viewMatrix = _camera->GetView();
	_projMatrix = _camera->GetProj();
														
}

void Scene::Draw()
{
	// Draw objects, giving the camera's position and projection
	_physics_object_1->Draw(_viewMatrix, _projMatrix);
	_physics_object_2->Draw(_viewMatrix, _projMatrix);
	_level->Draw(_viewMatrix, _projMatrix);

}


