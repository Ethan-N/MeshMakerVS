#include "Circles.h"


Circles::Circles() {
    // Initialize the geometry
    setGeometry(60, 1);
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
    mesh.setUsage(GL_STATIC_DRAW);
 
    buffer.allocate();
    buffer.bind(GL_TEXTURE_BUFFER);

    shader.load("vert.glsl", "frag.glsl");

    // then we tell the vbo that colors should be used per instance by using
    // ofVbo::setAttributeDivisor
    mesh.getVbo().setAttributeDivisor(ofShader::COLOR_ATTRIBUTE, 1);
    resize(10);
}

Circles::~Circles() {

}

void Circles::setGeometry(int resolution, float radius) {
    mesh.clearVertices();
    mesh.addVertex(glm::vec3(0, 0, 0));
    for (int i = 0; i < resolution; i++) {
        float v = i * PI * 2 / (resolution - 1);
        float x = cos(v) * radius;
        float y = sin(v) * radius;
        mesh.addVertex(glm::vec3(x, y, 0));
    }
};

void Circles::resize(size_t size) {
    matrices.resize(size, ofMatrix4x4(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0));
    buffer.setData(matrices, GL_STREAM_DRAW);
    tex.allocateAsBufferTexture(buffer, GL_RGBA32F); // https://www.opengl.org/wiki/Buffer_Texture

    // we want each box to have a different color so let's add
    // as many colors as boxes

    mesh.getColors().resize(matrices.size());
    for (size_t i = 0; i < mesh.getColors().size(); i++) {
        mesh.getColors()[i] = ofColor::white;
    }
    shader.begin();
    shader.setUniformTexture("tex", tex, 0);
    shader.end();
    ofLog() << matrices.size();
}

size_t Circles::size() {
    return matrices.size();
}

void Circles::setColor(size_t i, const ofColor color) {
    if (i < matrices.size()) mesh.getColors()[i] = color;
}

void Circles::setMatrix(size_t i, const glm::mat4 matrix) {
    if (i < matrices.size()) matrices[i] = matrix;
}

void Circles::clear() {
	for (int i = 0; i < matrices.size(); i++) {
		matrices[i] = ofMatrix4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	}
}

glm::mat4 Circles::getMatrix(size_t i) {
	return matrices[i];
}

void Circles::updateGpu() {
    buffer.updateData(matrices);
}

void Circles::draw() {
    shader.begin();
    mesh.drawInstanced(OF_MESH_FILL, matrices.size());
    shader.end();
}