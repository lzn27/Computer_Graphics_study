// 惨痛教训：glcolor3f()的参数返回在 [0.0 ~ 1.0] 而不是 [0.0 ~ 255.0]。
// 即：用浮点数表示RGB数据范围在0-1之间
// 扩展：glColor3ub (GLubyte red, GLubyte green, GLubyte blue) 范围是 [0 ~ 255]。
// glColor3i(int r, int g ,int b) 范围是 [0 ~ INT_MAX] -> #include<limit.h>。
// glColor3b (GLbyte red, GLbyte green, GLbyte blue) 范围是 [-128 ~ 127]。真坑爹。。
// 待深入：窗口移动或者缩放时，会自动调用Glut中默认的重绘回调函数，可能会绘制出不一样的图像，或许是因为缓冲区？
#include<vector>
#include "gl/glut.h"
#include "glm/glm.hpp"
using namespace std;

vector<vector<GLfloat> > vers{ {0.0,0.5},{0.1333,0.3},{0.4,0.3},{0.2667,0.0},{0.4,-0.3},{0.1333,-0.3},{0.0,-0.5},{-0.1333,-0.3},{-0.4,-0.3},{-0.2667,0.0},{-0.4,0.3},{-0.1333,0.3},{0.0,0.5} };
constexpr GLfloat pricision = 255.0;
const GLfloat rgbpricision = 1.0 * 6 / (12 * pricision);//六芒星12条边
vector<GLfloat> delta_v{ 0.0,rgbpricision,0.0 };
size_t index = 1;
GLfloat flag = 1, target = 1.0;

void nextRGB(vector<GLfloat>& v) {
    v[0] += delta_v[0];
    v[1] += delta_v[1];
    v[2] += delta_v[2];
    if (abs(v[index] - target) <= rgbpricision) {
        v[index] = target;
        target = abs(target) < 0.00001 ? 1.0 : 0.0;
        index = (index + 2) % 3;
        flag *= -1;
        delta_v[index] = rgbpricision * flag;
        delta_v[(index + 2) % 3] = 0.0;
        delta_v[((index + 2) % 3 + 2) % 3] = 0.0;
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    vector<GLfloat> starcolor{ 1.0,0.0,0.0 };
    glColor3f(starcolor[0], starcolor[1], starcolor[2]);
    glBegin(GL_LINES); 
    for (size_t count = 0; count < vers.size()-1; count++) {
        vector<GLfloat> s(vers[count]);
        vector<GLfloat> e(vers[count + 1]);
        GLfloat delta_x = (e[0] - s[0]) / pricision;
        GLfloat delta_y = (e[1] - s[1]) / pricision;
        while (abs(s[0] - e[0]) >= abs(delta_x)) {
            glVertex2f(0.0, 0.0);
            glVertex2f(s[0], s[1]);
            s[0] += delta_x;
            s[1] += delta_y;
            nextRGB(starcolor);
            glColor3f(starcolor[0], starcolor[1], starcolor[2]);
        }
    }
    glEnd(); 
    //glutSwapBuffers();
    glFlush();
}
int main(int argc,char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowPosition(100, 100);
    glutInitWindowSize(600, 600);
    glutCreateWindow("square");
    
    glClearColor(255.0, 255.0, 255.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);

    glutDisplayFunc(display);
    glutMainLoop();

    return 0;
}