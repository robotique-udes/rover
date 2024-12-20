/****************************************************************************
** Meta object code from reading C++ file 'QGStreamerWorker.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../rover_gui/src/QRtspPlayer/QGStreamerWorker.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QGStreamerWorker.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_GStreamerWorker_t {
    const uint offsetsAndSize[18];
    char stringdata0[104];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_GStreamerWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_GStreamerWorker_t qt_meta_stringdata_GStreamerWorker = {
    {
QT_MOC_LITERAL(0, 15), // "GStreamerWorker"
QT_MOC_LITERAL(16, 15), // "pipelineStarted"
QT_MOC_LITERAL(32, 0), // ""
QT_MOC_LITERAL(33, 15), // "pipelineStopped"
QT_MOC_LITERAL(49, 13), // "errorOccurred"
QT_MOC_LITERAL(63, 5), // "error"
QT_MOC_LITERAL(69, 13), // "startPipeline"
QT_MOC_LITERAL(83, 7), // "rtspUrl"
QT_MOC_LITERAL(91, 12) // "stopPipeline"

    },
    "GStreamerWorker\0pipelineStarted\0\0"
    "pipelineStopped\0errorOccurred\0error\0"
    "startPipeline\0rtspUrl\0stopPipeline"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GStreamerWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   44,    2, 0x06,    1 /* Public */,
       3,    0,   45,    2, 0x06,    2 /* Public */,
       4,    1,   46,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       6,    1,   49,    2, 0x0a,    5 /* Public */,
       8,    0,   52,    2, 0x0a,    7 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void,

       0        // eod
};

void GStreamerWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GStreamerWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->pipelineStarted(); break;
        case 1: _t->pipelineStopped(); break;
        case 2: _t->errorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->startPipeline((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->stopPipeline(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (GStreamerWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GStreamerWorker::pipelineStarted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (GStreamerWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GStreamerWorker::pipelineStopped)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (GStreamerWorker::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GStreamerWorker::errorOccurred)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject GStreamerWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_GStreamerWorker.offsetsAndSize,
    qt_meta_data_GStreamerWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_GStreamerWorker_t
, QtPrivate::TypeAndForceComplete<GStreamerWorker, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *GStreamerWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GStreamerWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GStreamerWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int GStreamerWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void GStreamerWorker::pipelineStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void GStreamerWorker::pipelineStopped()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void GStreamerWorker::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
