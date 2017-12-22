
#include <QtGui>
#include <QtOpenGL>
#include <math.h>
#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    object = 0;
    xRot = 0;
    yRot = 0;
    zRot = 0;

    trolltechGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
    trolltechPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);

}

GLWidget::~GLWidget()
{
    makeCurrent();
    glDeleteLists(object, 1);
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400, 400);
}

void GLWidget::setXRotation(int angle)
{
    normalizeAngle(&angle);
    if (angle != xRot) {
        xRot = angle;
        updateGL();
    }
}

void GLWidget::setYRotation(int angle)
{
    normalizeAngle(&angle);
    if (angle != yRot) {
        yRot = angle;
        updateGL();
    }
}

void GLWidget::setZRotation(int angle)
{
    normalizeAngle(&angle);
    if (angle != zRot) {
        zRot = angle;
        updateGL();
    }
}

void GLWidget::initializeGL()
{

    qglClearColor(trolltechPurple.dark());
    qglClearColor(QColor::fromRgb(0, 127, 200, 0));
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    object = makeObject();
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // light setup
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat specular[] = {1.0,1.0,1.0,1.0};
    //GLfloat specref[]= {1.0,1.0,1.0,1.0};
    GLfloat diffuse[] = {1.0,1.0,1.0,1.0};
    GLfloat ambient[] = {0.7,0.7,0.7,1.0};
    GLfloat pos[] = {0.0,0.0,50.0,0.0};
    glLightfv(GL_LIGHT0,GL_SPECULAR,specular);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
    glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
    glLightfv(GL_LIGHT0,GL_POSITION,pos);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    //glMaterialfv(GL_FRONT, GL_SPECULAR, specref);
    glMateriali(GL_FRONT, GL_SHININESS, 14);

}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -10.0);
    glRotated(90.0, 1.0, 0.0, 0.0); //makes the OpenGL coordinates system reflect the accelerometer coordinates system (y=front, x=right, z=up)
    glRotated(xRot, 1.0, 0.0, 0.0);
    glRotated(yRot, 0.0, 1.0, 0.0);
    glRotated(zRot, 0.0, 0.0, 1.0);
    //std::cerr << "x = " << xRot << std::endl;
    //std::cerr << "y = " << yRot << std::endl;
    //std::cerr << "z = " << zRot << std::endl;
    glCallList(object);

}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setZRotation(zRot + 8 * dx);
    }
    lastPos = event->pos();
}

GLuint GLWidget::makeObject()
{
    GLfloat black[] = {0.0,0.0,0.0};
    GLfloat darkGreen[] = {0.0, 0.5, 0.5};
    GLfloat darkGreen2[] = {0.0, 0.6, 0.6};
    GLfloat lightGrey[] = {0.8, 0.8, 0.8};
    GLfloat yellow[] = {1.0, 1.0, 0.0};
    int i = 0;

    GLuint list = glGenLists(1);
    glNewList(list, GL_COMPILE);

    // right wheel
    glPushMatrix();
    glTranslated(0.3, 0.0, 0.0);
    cylinder(0.24, 0.02, 1, 1, 20, black);
    glPopMatrix();

    // left wheel
    glPushMatrix();
    glTranslated(-0.32, 0.0, 0.0);
    cylinder(0.24, 0.02, 1, 1, 20, black);
    glPopMatrix();

    // body
    glPushMatrix();
    glRotated(90.0, 0.0, 0.0, 1.0);
    glRotated(90.0, 0.0, 1.0, 0.0);
    glTranslated(-0.21, 0.0, 0.0);
    cylinder(0.3, 0.4, 1, 1, 20, lightGrey);
    glPopMatrix();

    // pcb on top of body
    glPushMatrix();
    glRotated(90.0, 0.0, 0.0, 1.0);
    glRotated(90.0, 0.0, 1.0, 0.0);
    glTranslated(-0.23, 0.0, 0.0);
    cylinder(0.34, 0.02, 1, 1, 20, darkGreen);
    glPopMatrix();

    // distanziatori
    glPushMatrix();
    glRotated(90.0, 0.0, 0.0, 1.0);
    glRotated(90.0, 0.0, 1.0, 0.0);
    glTranslated(-0.28, 0.0, 0.0);
    glPushMatrix();
    glTranslated(0.0, 0.0, 0.3);
    cylinder(0.02, 0.05, 1, 1, 20, yellow);
    glPopMatrix();
    glPushMatrix();
    glTranslated(0.0, 0.2, -0.2);
    cylinder(0.02, 0.05, 1, 1, 20, yellow);
    glPopMatrix();
    glPushMatrix();
    glTranslated(0.0, -0.2, -0.2);
    cylinder(0.02, 0.05, 1, 1, 20, yellow);
    glPopMatrix();
    glPopMatrix();

    // pcb jumper
    glColor3fv(darkGreen2);
    glPushMatrix();
    glRotated(90.0, 1.0, 0.0, 0.0);
    glTranslated(0.0, 0.28, 0.0);
    glRotated(180.0, 0.0, 1.0, 0.0);
    glBegin(GL_POLYGON);
    glVertex3f(0.0, 0.0, -0.13);
    glVertex3f(0.08, 0.0, -0.11);
    glVertex3f(0.2, 0.0, -0.2);
    glVertex3f(0.1, 0.0, -0.1);
    glVertex3f(0.2, 0.0, 0.28);
    glVertex3f(0.0, 0.0, 0.3);
    glVertex3f(-0.2, 0.0, 0.28);
    glVertex3f(-0.1, 0.0, -0.1);
    glVertex3f(-0.2, 0.0, -0.2);
    glVertex3f(-0.08, 0.0, -0.11);
    glEnd();
    glPopMatrix();

    glEndList();
    return list;
}

void GLWidget::quad(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble x3, GLdouble y3, GLdouble x4, GLdouble y4, GLdouble size)
{
    qglColor(trolltechGreen);

    glVertex3d(x1, y1, -size);
    glVertex3d(x2, y2, -size);
    glVertex3d(x3, y3, -size);
    glVertex3d(x4, y4, -size);

    glVertex3d(x4, y4, +size);
    glVertex3d(x3, y3, +size);
    glVertex3d(x2, y2, +size);
    glVertex3d(x1, y1, +size);
}

void GLWidget::extrude(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble size)
{
    qglColor(trolltechGreen.dark(250 + int(100 * x1)));

    glVertex3d(x1, y1, +size);
    glVertex3d(x2, y2, +size);
    glVertex3d(x2, y2, -size);
    glVertex3d(x1, y1, -size);
}

void GLWidget::normalizeAngle(int *angle)
{
    while (*angle < 0)
        *angle += 360;
    while (*angle > 360)
        *angle -= 360;
}

void GLWidget::cylinder(double r, double h, int f1, int f2, int risol, GLfloat colore[]) {
  int i,j;
  double ris=(double) risol;
  double cos_alfa = cos(PI/ris);
  double sin_alfa = sin(PI/ris);

  glColor3fv(colore);
  //glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
  //Perimetro
  glPushMatrix();
    for(i=0;i<2*(int)ris;i++){
      glBegin(GL_QUADS);
        glNormal3f(0.0,r,0.0);
        glVertex3f((GLfloat)h,(GLfloat)r,0.0);
        glVertex3f(0.0,(GLfloat)r,0.0);
        glNormal3f(0.0,(GLfloat)(cos_alfa*r),(GLfloat)(sin_alfa*r));
        glVertex3f(0.0,(GLfloat)(cos_alfa*r),(GLfloat)(sin_alfa*r));
        glVertex3f((GLfloat)h,(GLfloat)(cos_alfa*r),(GLfloat)(sin_alfa*r));
      glEnd();
      glRotated(180.0/ris,1.0,0.0,0.0);
    }
  glPopMatrix();

  // FACCIA 1
  if(f1){
    glPushMatrix();
      glNormal3f(1.0,0.0,0.0);
      for(i=0.0;i<2*(int)ris;i++){
        glBegin(GL_TRIANGLES);
          glVertex3f((GLfloat)h,0.0,0.0);
          glVertex3f((GLfloat)h,(GLfloat)(sin_alfa*r),(GLfloat)(cos_alfa*r));
          glVertex3f((GLfloat)h,0.0,(GLfloat)r);
        glEnd();
        glRotated(180.0/ris,1.0,0.0,0.0);
      }
    glPopMatrix();
  }
  // FACCIA 2
  if(f2){
    glPushMatrix();
      glNormal3f(-1.0,0.0,0.0);
      for(i=0.0;i<2*(int)ris;i++){
        glBegin(GL_TRIANGLES);
          glVertex3f(0.0,0.0,0.0);
          glVertex3f(0.0,0.0,(GLfloat)r);
          glVertex3f(0.0,(GLfloat)(sin_alfa*r),(GLfloat)(cos_alfa*r));
        glEnd();
        glRotated(180.0/ris,1.0,0.0,0.0);
      }
    glPopMatrix();
  }
}
