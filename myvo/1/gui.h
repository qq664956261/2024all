#include "imgui/imgui.h"
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
class gui
{

public:
    gui();
    ~gui();
    GLuint MatToTexture(const cv::Mat &mat);
};


