/****************************************************************************
** Meta object code from reading C++ file 'QSshWorker.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/QSshFileExplorer/Worker/QSshWorker.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QSshWorker.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QSshWorker_t {
    const uint offsetsAndSize[14];
    char stringdata0[97];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_QSshWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_QSshWorker_t qt_meta_stringdata_QSshWorker = {
    {
QT_MOC_LITERAL(0, 10), // "QSshWorker"
QT_MOC_LITERAL(11, 17), // "newStructureReady"
QT_MOC_LITERAL(29, 0), // ""
QT_MOC_LITERAL(30, 20), // "newProgressBarUpdate"
QT_MOC_LITERAL(51, 11), // "std::string"
QT_MOC_LITERAL(63, 16), // "taskDescription_"
QT_MOC_LITERAL(80, 16) // "progressPercent_"

    },
    "QSshWorker\0newStructureReady\0\0"
    "newProgressBarUpdate\0std::string\0"
    "taskDescription_\0progressPercent_"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QSshWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   26,    2, 0x06,    1 /* Public */,
       3,    2,   27,    2, 0x06,    2 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4, QMetaType::Float,    5,    6,

       0        // eod
};

void QSshWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QSshWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->newStructureReady(); break;
        case 1: _t->newProgressBarUpdate((*reinterpret_cast< std::add_pointer_t<std::string>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QSshWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QSshWorker::newStructureReady)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QSshWorker::*)(std::string , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QSshWorker::newProgressBarUpdate)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject QSshWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<QWorker::staticMetaObject>(),
    qt_meta_stringdata_QSshWorker.offsetsAndSize,
    qt_meta_data_QSshWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_QSshWorker_t
, QtPrivate::TypeAndForceComplete<QSshWorker, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::string, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>



>,
    nullptr
} };


const QMetaObject *QSshWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QSshWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QSshWorker.stringdata0))
        return static_cast<void*>(this);
    return QWorker::qt_metacast(_clname);
}

int QSshWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void QSshWorker::newStructureReady()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void QSshWorker::newProgressBarUpdate(std::string _t1, float _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
