/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Sat Jan 11 05:22:40 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      20,   11,   11,   11, 0x0a,
      28,   11,   11,   11, 0x0a,
      36,   11,   11,   11, 0x0a,
      44,   11,   11,   11, 0x0a,
      51,   11,   11,   11, 0x0a,
      58,   11,   11,   11, 0x0a,
      65,   11,   11,   11, 0x0a,
      72,   11,   11,   11, 0x0a,
      79,   11,   11,   11, 0x0a,
      86,   11,   11,   11, 0x0a,
      99,   11,   11,   11, 0x0a,
     125,  112,   11,   11, 0x0a,
     160,   11,   11,   11, 0x0a,
     174,   11,   11,   11, 0x0a,
     188,   11,   11,   11, 0x0a,
     202,   11,   11,   11, 0x0a,
     218,   11,   11,   11, 0x0a,
     230,  224,   11,   11, 0x0a,
     277,  260,   11,   11, 0x08,
     345,   11,   11,   11, 0x08,
     371,   11,   11,   11, 0x08,
     397,   11,   11,   11, 0x08,
     409,   11,   11,   11, 0x08,
     435,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0grmax()\0grmin()\0brmin()\0"
    "brmax()\0ymin()\0ymax()\0umin()\0umax()\0"
    "vmin()\0vmax()\0rgbclicked()\0yuvclicked()\0"
    "iplImg,image\0IplImage2QImage(IplImage*,QImage*)\0"
    "saveclicked()\0loadclicked()\0quitclicked()\0"
    "lookupclicked()\0asd()\0event\0"
    "mousePressEvent(QMouseEvent*)\0"
    "current,previous\0"
    "on_listWidget_currentItemChanged(QListWidgetItem*,QListWidgetItem*)\0"
    "on_pushButton_3_clicked()\0"
    "on_pushButton_4_clicked()\0onTimeout()\0"
    "on_pushButton_5_clicked()\0"
    "on_pushButton_6_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->grmax(); break;
        case 1: _t->grmin(); break;
        case 2: _t->brmin(); break;
        case 3: _t->brmax(); break;
        case 4: _t->ymin(); break;
        case 5: _t->ymax(); break;
        case 6: _t->umin(); break;
        case 7: _t->umax(); break;
        case 8: _t->vmin(); break;
        case 9: _t->vmax(); break;
        case 10: _t->rgbclicked(); break;
        case 11: _t->yuvclicked(); break;
        case 12: _t->IplImage2QImage((*reinterpret_cast< IplImage*(*)>(_a[1])),(*reinterpret_cast< QImage*(*)>(_a[2]))); break;
        case 13: _t->saveclicked(); break;
        case 14: _t->loadclicked(); break;
        case 15: _t->quitclicked(); break;
        case 16: _t->lookupclicked(); break;
        case 17: _t->asd(); break;
        case 18: _t->mousePressEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 19: _t->on_listWidget_currentItemChanged((*reinterpret_cast< QListWidgetItem*(*)>(_a[1])),(*reinterpret_cast< QListWidgetItem*(*)>(_a[2]))); break;
        case 20: _t->on_pushButton_3_clicked(); break;
        case 21: _t->on_pushButton_4_clicked(); break;
        case 22: _t->onTimeout(); break;
        case 23: _t->on_pushButton_5_clicked(); break;
        case 24: _t->on_pushButton_6_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
