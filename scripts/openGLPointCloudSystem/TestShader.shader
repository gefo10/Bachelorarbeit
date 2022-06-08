#shader vertex
#version 330 core

layout(location = 0) in vec4 position;

uniform mat4 MVP;
uniform float size;

void main()
{
    gl_PointSize = size;
    gl_Position = MVP * position;
};

#shader fragment
#version 330 core

out vec4 color;

void main()
{
    color = vec4(1.0,0.0,0.0,1.0);
};

