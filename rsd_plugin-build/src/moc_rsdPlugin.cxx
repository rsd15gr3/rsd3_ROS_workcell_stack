/****************************************************************************
** Meta object code from reading C++ file 'rsdPlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../rsd_plugin/src/rsdPlugin.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rsdPlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rsdPluginPlugin[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   16,   16,   16, 0x05,

 // slots: signature, parameters, type, tag, flags
      27,   16,   16,   16, 0x08,
      40,   16,   16,   16, 0x08,
      54,   48,   16,   16, 0x08,
      98,   16,   16,   16, 0x08,
     116,   16,   16,   16, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_rsdPluginPlugin[] = {
    "rsdPluginPlugin\0\0quitNow()\0btnPressed()\0"
    "timer()\0state\0stateChangedListener(rw::kinematics::State)\0"
    "newImage(cv::Mat)\0"
    "updateConfiguration(kuka_rsi::getConfiguration)\0"
};

void rsdPluginPlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        rsdPluginPlugin *_t = static_cast<rsdPluginPlugin *>(_o);
        switch (_id) {
        case 0: _t->quitNow(); break;
        case 1: _t->btnPressed(); break;
        case 2: _t->timer(); break;
        case 3: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        case 4: _t->newImage((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 5: _t->updateConfiguration((*reinterpret_cast< kuka_rsi::getConfiguration(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData rsdPluginPlugin::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rsdPluginPlugin::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_rsdPluginPlugin,
      qt_meta_data_rsdPluginPlugin, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rsdPluginPlugin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rsdPluginPlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rsdPluginPlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rsdPluginPlugin))
        return static_cast<void*>(const_cast< rsdPluginPlugin*>(this));
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(const_cast< rsdPluginPlugin*>(this));
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int rsdPluginPlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void rsdPluginPlugin::quitNow()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
