/****************************************************************************
** Meta object code from reading C++ file 'SecondaryWindow.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/SecondaryWindow.hpp"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SecondaryWindow.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SecondaryWindow_t {
    const uint offsetsAndSize[22];
    char stringdata0[131];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_SecondaryWindow_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_SecondaryWindow_t qt_meta_stringdata_SecondaryWindow = {
    {
QT_MOC_LITERAL(0, 15), // "SecondaryWindow"
QT_MOC_LITERAL(16, 14), // "onLayoutChange"
QT_MOC_LITERAL(31, 0), // ""
QT_MOC_LITERAL(32, 5), // "index"
QT_MOC_LITERAL(38, 9), // "addStream"
QT_MOC_LITERAL(48, 10), // "editStream"
QT_MOC_LITERAL(59, 12), // "removeStream"
QT_MOC_LITERAL(72, 17), // "onStreamSelection"
QT_MOC_LITERAL(90, 20), // "onStreamStateChanged"
QT_MOC_LITERAL(111, 7), // "running"
QT_MOC_LITERAL(119, 11) // "streamIndex"

    },
    "SecondaryWindow\0onLayoutChange\0\0index\0"
    "addStream\0editStream\0removeStream\0"
    "onStreamSelection\0onStreamStateChanged\0"
    "running\0streamIndex"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SecondaryWindow[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   50,    2, 0x08,    1 /* Private */,
       4,    0,   53,    2, 0x08,    3 /* Private */,
       5,    0,   54,    2, 0x08,    4 /* Private */,
       6,    0,   55,    2, 0x08,    5 /* Private */,
       7,    1,   56,    2, 0x08,    6 /* Private */,
       8,    2,   59,    2, 0x08,    8 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,    9,   10,

       0        // eod
};

void SecondaryWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SecondaryWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onLayoutChange((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 1: _t->addStream(); break;
        case 2: _t->editStream(); break;
        case 3: _t->removeStream(); break;
        case 4: _t->onStreamSelection((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 5: _t->onStreamStateChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObject SecondaryWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_SecondaryWindow.offsetsAndSize,
    qt_meta_data_SecondaryWindow,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_SecondaryWindow_t
, QtPrivate::TypeAndForceComplete<SecondaryWindow, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>


>,
    nullptr
} };


const QMetaObject *SecondaryWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SecondaryWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SecondaryWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int SecondaryWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
