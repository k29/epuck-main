/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      28,   11,   11,   11, 0x05,
      45,   11,   11,   11, 0x05,
      62,   56,   11,   11, 0x05,
      79,   56,   11,   11, 0x05,
     110,  106,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     131,  128,   11,   11, 0x08,
     157,   11,   11,   11, 0x08,
     186,   11,   11,   11, 0x08,
     216,  214,   11,   11, 0x08,
     237,   56,   11,   11, 0x08,
     278,   56,   11,   11, 0x08,
     329,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0stopCamThread()\0"
    "stopAlgoThread()\0stopAlgo()\0index\0"
    "algoChanged(int)\0algoActivationChanged(int)\0"
    "val\0resetClicked(int)\0pm\0"
    "onCamImageReady(QPixmap*)\0"
    "on_algoStartButton_clicked()\0"
    "on_algoStopButton_clicked()\0s\0"
    "onGetResult(QString)\0"
    "on_algoComboBox_currentIndexChanged(int)\0"
    "on_algoActivationComboBox_currentIndexChanged(int)\0"
    "on_resetButton_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->stopCamThread(); break;
        case 1: _t->stopAlgoThread(); break;
        case 2: _t->stopAlgo(); break;
        case 3: _t->algoChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->algoActivationChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->resetClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->onCamImageReady((*reinterpret_cast< QPixmap*(*)>(_a[1]))); break;
        case 7: _t->on_algoStartButton_clicked(); break;
        case 8: _t->on_algoStopButton_clicked(); break;
        case 9: _t->onGetResult((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->on_algoComboBox_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_algoActivationComboBox_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_resetButton_clicked(); break;
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
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::stopCamThread()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void MainWindow::stopAlgoThread()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void MainWindow::stopAlgo()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void MainWindow::algoChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MainWindow::algoActivationChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void MainWindow::resetClicked(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
