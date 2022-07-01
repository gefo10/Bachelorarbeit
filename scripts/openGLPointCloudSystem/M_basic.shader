#shader vertex
#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

out vec3 color_from_vb;

uniform mat4 MVP;
void main()
{
    gl_Position = MVP * vec4(position,1.0);
    color_from_vb = color;
};

#shader fragment
#version 330 core

in vec3 color_from_vb;
out vec4 color_fs;

void main()
{
    color_fs = vec4(color_from_vb,1.0f);
};


