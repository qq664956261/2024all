#include "gui.h"
gui::gui()
{
    // 初始化 GLFW 和 OpenGL
    if (!glfwInit())
    {

    }
    GLFWwindow *window = glfwCreateWindow(1280, 720, "Feature Tracker with ImGui", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // // 初始化 ImGui
    ImGui::CreateContext();
    // ImGui_ImplGlfw_InitForOpenGL(window, true);
    // ImGui_ImplOpenGL3_Init("#version 130");
}

gui::~gui()
{
}

// OpenCV 图像转换为 OpenGL 纹理的函数
GLuint gui::MatToTexture(const cv::Mat &mat)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // OpenCV 默认是 BGR，需转换为 RGB
    cv::Mat rgbMat;
    cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgbMat.cols, rgbMat.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgbMat.data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return textureID;
}