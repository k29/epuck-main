/****************************************************************************
** Meta object code from reading C++ file 'cameraworker.h'
**
** Created: Tue Aug 26 16:00:17 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cameraworker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cameraworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CameraWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x05,
      29,   25,   13,   13, 0x05,
      47,   44,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      68,   13,   13,   13, 0x0a,
      78,   13,   13,   13, 0x0a,
      90,   13,   13,   13, 0x0a,
     111,   99,   13,   13, 0x0a,
     140,  138,   13,   13, 0x0a,
     172,  170,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_CameraWorker[] = {
    "CameraWorker\0\0finished()\0err\0"
    "error(QString)\0pm\0imageReady(QPixmap*)\0"
    "process()\0onTimeout()\0onStop()\0"
    "x1,y1,x2,y2\0onGotLine(int,int,int,int)\0"
    "p\0onPrintDestination(PointList)\0c\0"
    "onPrintCircle(Circle)\0"
};

void CameraWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CameraWorker *_t = static_cast<CameraWorker *>(_o);
        switch (_id) {
        case 0: _t->finished(); break;
        case 1: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->imageReady((*reinterpret_cast< QPixmap*(*)>(_a[1]))); break;
        case 3: _t->process(); break;
        case 4: _t->onTimeout(); break;
        case 5: _t->onStop(); break;
        case 6: _t->onGotLine((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 7: _t->onPrintDestination((*reinterpret_cast< PointList(*)>(_a[1]))); break;
        case 8: _t->onPrintCircle((*reinterpret_cast< Circle(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CameraWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CameraWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_CameraWorker,
      qt_meta_data_CameraWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CameraWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CameraWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CameraWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CameraWorker))
        return static_cast<void*>(const_cast< CameraWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int CameraWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void CameraWorker::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void CameraWorker::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CameraWorker::imageReady(QPixmap * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
