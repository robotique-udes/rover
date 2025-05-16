/****************************************************************************
** Meta object code from reading C++ file 'QRtspPlayerWidget.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/QRtspPlayer/QRtspPlayerWidget.hpp"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QRtspPlayerWidget.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RtspPlayerWidget_t {
    const uint offsetsAndSize[46];
    char stringdata0[286];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_RtspPlayerWidget_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_RtspPlayerWidget_t qt_meta_stringdata_RtspPlayerWidget = {
    {
QT_MOC_LITERAL(0, 16), // "RtspPlayerWidget"
QT_MOC_LITERAL(17, 18), // "requestStartStream"
QT_MOC_LITERAL(36, 0), // ""
QT_MOC_LITERAL(37, 7), // "rtspUrl"
QT_MOC_LITERAL(45, 17), // "requestStopStream"
QT_MOC_LITERAL(63, 18), // "streamStateChanged"
QT_MOC_LITERAL(82, 9), // "isRunning"
QT_MOC_LITERAL(92, 11), // "streamIndex"
QT_MOC_LITERAL(104, 17), // "onPipelineStarted"
QT_MOC_LITERAL(122, 11), // "GstElement*"
QT_MOC_LITERAL(134, 8), // "pipeline"
QT_MOC_LITERAL(143, 15), // "onErrorOccurred"
QT_MOC_LITERAL(159, 5), // "error"
QT_MOC_LITERAL(165, 15), // "onNewLogMessage"
QT_MOC_LITERAL(181, 7), // "message"
QT_MOC_LITERAL(189, 6), // "target"
QT_MOC_LITERAL(196, 13), // "onToggleDebug"
QT_MOC_LITERAL(210, 7), // "checked"
QT_MOC_LITERAL(218, 12), // "onToggleInfo"
QT_MOC_LITERAL(231, 15), // "onToggleWarning"
QT_MOC_LITERAL(247, 13), // "onToggleError"
QT_MOC_LITERAL(261, 11), // "onClearLogs"
QT_MOC_LITERAL(273, 12) // "onToggleView"

    },
    "RtspPlayerWidget\0requestStartStream\0"
    "\0rtspUrl\0requestStopStream\0"
    "streamStateChanged\0isRunning\0streamIndex\0"
    "onPipelineStarted\0GstElement*\0pipeline\0"
    "onErrorOccurred\0error\0onNewLogMessage\0"
    "message\0target\0onToggleDebug\0checked\0"
    "onToggleInfo\0onToggleWarning\0onToggleError\0"
    "onClearLogs\0onToggleView"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RtspPlayerWidget[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   86,    2, 0x06,    1 /* Public */,
       4,    0,   89,    2, 0x06,    3 /* Public */,
       5,    2,   90,    2, 0x06,    4 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       8,    1,   95,    2, 0x08,    7 /* Private */,
      11,    1,   98,    2, 0x08,    9 /* Private */,
      13,    2,  101,    2, 0x08,   11 /* Private */,
      16,    1,  106,    2, 0x08,   14 /* Private */,
      18,    1,  109,    2, 0x08,   16 /* Private */,
      19,    1,  112,    2, 0x08,   18 /* Private */,
      20,    1,  115,    2, 0x08,   20 /* Private */,
      21,    0,  118,    2, 0x08,   22 /* Private */,
      22,    0,  119,    2, 0x08,   23 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,    6,    7,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   14,   15,
    QMetaType::Void, QMetaType::Bool,   17,
    QMetaType::Void, QMetaType::Bool,   17,
    QMetaType::Void, QMetaType::Bool,   17,
    QMetaType::Void, QMetaType::Bool,   17,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RtspPlayerWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RtspPlayerWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->requestStartStream((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->requestStopStream(); break;
        case 2: _t->streamStateChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 3: _t->onPipelineStarted((*reinterpret_cast< std::add_pointer_t<GstElement*>>(_a[1]))); break;
        case 4: _t->onErrorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 5: _t->onNewLogMessage((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 6: _t->onToggleDebug((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 7: _t->onToggleInfo((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 8: _t->onToggleWarning((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 9: _t->onToggleError((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 10: _t->onClearLogs(); break;
        case 11: _t->onToggleView(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RtspPlayerWidget::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RtspPlayerWidget::requestStartStream)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (RtspPlayerWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RtspPlayerWidget::requestStopStream)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (RtspPlayerWidget::*)(bool , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RtspPlayerWidget::streamStateChanged)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject RtspPlayerWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_RtspPlayerWidget.offsetsAndSize,
    qt_meta_data_RtspPlayerWidget,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_RtspPlayerWidget_t
, QtPrivate::TypeAndForceComplete<RtspPlayerWidget, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<GstElement *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *RtspPlayerWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RtspPlayerWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RtspPlayerWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int RtspPlayerWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void RtspPlayerWidget::requestStartStream(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void RtspPlayerWidget::requestStopStream()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void RtspPlayerWidget::streamStateChanged(bool _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
