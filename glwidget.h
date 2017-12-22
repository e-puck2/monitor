
#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <iostream>

#define PI 3.1415926535897932384626433832795

class GLWidget : public QGLWidget
{
    Q_OBJECT

    public:
        GLWidget(QWidget *parent = 0);
        ~GLWidget();

        QSize minimumSizeHint() const;
        QSize sizeHint() const;

    public slots:
        void setXRotation(int angle);
        void setYRotation(int angle);
        void setZRotation(int angle);

    protected:
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void mousePressEvent(QMouseEvent *event);
        void mouseMoveEvent(QMouseEvent *event);

    private:
        GLuint makeObject();
        void quad(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble x3, GLdouble y3, GLdouble x4, GLdouble y4, GLdouble size);
        void extrude(GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, GLdouble size);
        void normalizeAngle(int *angle);
        void cylinder(double r, double h, int f1, int f2, int risol, GLfloat colore[]);

        GLuint object;
        int xRot;
        int yRot;
        int zRot;
        QPoint lastPos;
        QColor trolltechGreen;
        QColor trolltechPurple;
};

#endif
