#shader vertex
#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 color;
out vec3 color_from_vshader;

uniform mat4 MVP;

void main()
{
    gl_Position = MVP * position;
    color_from_vshader = color;
};

#shader fragment
#version 330 core

in vec3 color_from_vshader;
out vec4 color;

void main()
{
    color = vec4(color_from_vshader,1.0f);
};
