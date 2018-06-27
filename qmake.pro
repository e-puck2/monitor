TEMPLATE	= 	app
TARGET		=	EPuckMonitor
DESTDIR		= 	./
HEADERS		+= 	EpuckMonitor.h CommThread.h glwidget.h
SOURCES		+= 	main.cpp EpuckMonitor.cpp CommThread.cpp glwidget.cpp
win32 {
    LIBS    += -lopengl32
}
unix {

}

QT 		+= core opengl widgets network
FORMS		+= main.ui
RESOURCES       += resources.qrc
