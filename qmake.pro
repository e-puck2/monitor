TEMPLATE	= 	app
TARGET		=	EPuckMonitor
DESTDIR		= 	./
HEADERS		+= 	EpuckMonitor.h CommThread.h glwidget.h
SOURCES		+= 	main.cpp EpuckMonitor.cpp CommThread.cpp glwidget.cpp
win32 {
    SOURCES += comm.cpp
    HEADERS += comm.h
    LIBS    += -lopengl32
}
unix {
    SOURCES += SerialComm.cpp
    HEADERS += SerialComm.h
}

QT 		+= core opengl widgets
FORMS		+= main.ui
RESOURCES       += resources.qrc
