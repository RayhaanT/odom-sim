# Odometry Simulation

This project replicates the motion of a mobile robot with a holonomic drive for the purpose of testing position tracking code and motion algorithms.

## Tech
- [OpenGL](https://www.opengl.org/) - Graphics pipeline to render models
- [GLFW](https://www.glfw.org/) - OpenGL function library
- [GLAD](https://github.com/Dav1dde/glad) - OpenGL loading library, loads function pointers at runtime
- [GLM](https://glm.g-truc.net/0.9.8/index.html) - Math library used for graphics
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - Another math library, this one used to handle spline equations

## Building for Linux/MacOS
The included makefile will compile the project into the executable `simulation` using GNU make and gcc.

## Building for Windows
Using GNU's C++ compiler (g++), the following command can be used to compile the source:  
```g++ -g src/*.cpp src/*.c -static -Iinclude -iquote include -Llib -lopengl32 -lglfw3 -lgdi32 -o odom -mincoming-stack-boundary=2```  
Or, run buildAndRun.bat to compile and launch the executable:  
```./buildAndRun.bat```

## Acknowledgements
- Shaders and header files under `include/OpenGLHeaders` are derivative of samples from Joey de Vrie's [OpenGL tutorial series](https://learnopengl.com/Introduction) used under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
