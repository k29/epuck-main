/****************************************************************************
** Meta object code from reading C++ file 'algoworker.h'
**
** Created: Sat Aug 23 01:19:11 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "algoworker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'algoworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_AlgoWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      24,   12,   11,   11, 0x05,
      51,   49,   11,   11, 0x05,
      81,   79,   11,   11, 0x05,
     101,   11,   11,   11, 0x05,
     116,  112,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     131,   11,   11,   11, 0x0a,
     141,   11,   11,   11, 0x0a,
     153,   11,   11,   11, 0x0a,
     162,   11,   11,   11, 0x0a,
     176,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_AlgoWorker[] = {
    "AlgoWorker\0\0x1,y1,x2,y2\0"
    "gotLine(int,int,int,int)\0p\0"
    "printDestination(PointList)\0c\0"
    "printCircle(Circle)\0finished()\0err\0"
    "error(QString)\0process()\0onTimeout()\0"
    "onStop()\0onStartAlgo()\0onStopAlgo()\0"
};

void AlgoWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AlgoWorker *_t = static_cast<AlgoWorker *>(_o);
        switch (_id) {
        case 0: _t->gotLine((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 1: _t->printDestination((*reinterpret_cast< PointList(*)>(_a[1]))); break;
        case 2: _t->printCircle((*reinterpret_cast< Circle(*)>(_a[1]))); break;
        case 3: _t->finished(); break;
        case 4: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->process(); break;
        case 6: _t->onTimeout(); break;
        case 7: _t->onStop(); break;
        case 8: _t->onStartAlgo(); break;
        case 9: _t->onStopAlgo(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData AlgoWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject AlgoWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_AlgoWorker,
      qt_meta_data_AlgoWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &AlgoWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *AlgoWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *AlgoWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_AlgoWorker))
        return static_cast<void*>(const_cast< AlgoWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int AlgoWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void AlgoWorker::gotLine(int _t1, int _t2, int _t3, int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void AlgoWorker::printDestination(PointList _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void AlgoWorker::printCircle(Circle _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void AlgoWorker::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void AlgoWorker::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
