TEMPLATE	= 	app
TARGET		=	EPuckMonitor
DESTDIR		= 	./
LIBS		+= 	-dead_strip
HEADERS		+= 	EpuckMonitor.h CommThread.h glwidget.h
SOURCES		+= 	main.cpp EpuckMonitor.cpp CommThread.cpp glwidget.cpp
win32 {
     SOURCES += comm.cpp
	HEADERS+= comm.h
    LIBS += -static-libgcc
}
unix {
     SOURCES += SerialComm.cpp
	HEADERS+= SerialComm.h
}

CONFIG 		+= 	static separate_debug_info
QT 		= 	gui core
FORMS		+= 	main.ui
#DEFINES 	-= 	UNICODE
QT += opengl
RESOURCES += resources.qrc
