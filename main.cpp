#include <iostream>
#include <GLFW/glfw3.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

int SCREEN_WIDTH = 1000.0;
int SCREEN_HEIGHT = 1000.0;
int LIDAR_DATA_SIZE = 8192;
double SCALE_VALUE = 5.0;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
    if (key == GLFW_KEY_E && action == GLFW_PRESS)
	SCALE_VALUE -= 1.0;
    if (key == GLFW_KEY_Q && action == GLFW_PRESS)
    	SCALE_VALUE += 1.0;
}

struct node{
	double dist;
	double angle; //Radians
};

void processPoints(std::vector<double>& x, std::vector<double>& y, GLFWwindow* window){
	glPointSize(5);
	for(int i = 0; i < LIDAR_DATA_SIZE; i++){
		glBegin(GL_POINTS);
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex2f(x[i]/SCALE_VALUE, y[i]/SCALE_VALUE);
		glEnd();
	}
}

int main(){
	std::srand(std::time(0));
  	 GLFWwindow* window;
    /* Initialize the library */
    if (!glfwInit())
        return -1;
	/* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Hello World", NULL, NULL);
    if (!window){
        glfwTerminate();
        return -1;
}
    /* Make the window's context current */
	glfwMakeContextCurrent(window);
	glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
	glViewport(0,0,SCREEN_WIDTH,SCREEN_HEIGHT);
	glfwSetKeyCallback(window, key_callback);

    /* Loop until the user closes the window */
	nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
	inst.StartClient4("LIDARVisualizer");
	inst.SetServerTeam(5232);
	inst.StartDSClient();

	std::shared_ptr<nt::NetworkTable> lidarTable= inst.GetTable("lidar");
	
	nt::DoubleArrayTopic xTopic = lidarTable->GetDoubleArrayTopic("X");
	nt::DoubleArrayTopic yTopic = lidarTable->GetDoubleArrayTopic("Y");
	nt::DoubleTopic pidgeonTopic = lidarTable->GetDoubleTopic("pidgeon");

	std::vector<double> defaultArray = {0.0};

	nt::DoubleArraySubscriber xSub = xTopic.Subscribe(defaultArray);
	nt::DoubleArraySubscriber ySub = yTopic.Subscribe(defaultArray);
	nt::DoublePublisher pidgeonPub = pidgeonTopic.Publish();
	
	double pidgeonGyro = 1.0;

	std::vector<double> x(LIDAR_DATA_SIZE, 0.0);

	std::vector<double> y(LIDAR_DATA_SIZE, 0.0);


	while(!glfwWindowShouldClose(window)){
		x = xSub.Get();
		y = ySub.Get();

		pidgeonPub.Set(pidgeonGyro);
		//pidgeonGyro += .01;
		if(pidgeonGyro >= 360)
			pidgeonGyro = 0;
		glfwGetWindowSize(window, &SCREEN_WIDTH, &SCREEN_HEIGHT);
		
		glClear(GL_COLOR_BUFFER_BIT);

		processPoints(x, y, window);

		/* Render here */
        /* Swap front and back buffers */
		glfwSwapBuffers(window); 
        /* Poll for and process events */
        	glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}
