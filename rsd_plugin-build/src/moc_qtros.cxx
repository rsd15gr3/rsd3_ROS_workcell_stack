/****************************************************************************
** Meta object code from reading C++ file 'qtros.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../rsd_plugin/src/qtros.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qtros.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtROS[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,
      18,    6,    6,    6, 0x05,
      36,    6,    6,    6, 0x05,

 // slots: signature, parameters, type, tag, flags
      84,    6,    6,    6, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtROS[] = {
    "QtROS\0\0rosQuits()\0newImage(cv::Mat)\0"
    "updateConfiguration(kuka_rsi::getConfiguration)\0"
    "quitNow()\0"
};

void QtROS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtROS *_t = static_cast<QtROS *>(_o);
        switch (_id) {
        case 0: _t->rosQuits(); break;
        case 1: _t->newImage((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 2: _t->updateConfiguration((*reinterpret_cast< kuka_rsi::getConfiguration(*)>(_a[1]))); break;
        case 3: _t->quitNow(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtROS::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtROS::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_QtROS,
      qt_meta_data_QtROS, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtROS::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtROS::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtROS))
        return static_cast<void*>(const_cast< QtROS*>(this));
    return QThread::qt_metacast(_clname);
}

int QtROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void QtROS::rosQuits()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void QtROS::newImage(cv::Mat _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QtROS::updateConfiguration(kuka_rsi::getConfiguration _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
