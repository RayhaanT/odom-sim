g++ -g src/*.cpp src/*.c -static -Iinclude -iquote include -Llib -lopengl32 -lglfw3 -lgdi32 -o odom -mincoming-stack-boundary=2
odom.exe