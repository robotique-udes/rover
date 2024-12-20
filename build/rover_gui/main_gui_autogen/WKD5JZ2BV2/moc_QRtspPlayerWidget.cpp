/****************************************************************************
** Meta object code from reading C++ file 'QRtspPlayerWidget.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../rover_gui/src/QRtspPlayer/QRtspPlayerWidget.hpp"
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
    const uint offsetsAndSize[20];
    char stringdata0[124];
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
QT_MOC_LITERAL(63, 17), // "onPipelineStarted"
QT_MOC_LITERAL(81, 11), // "GstElement*"
QT_MOC_LITERAL(93, 8), // "pipeline"
QT_MOC_LITERAL(102, 15), // "onErrorOccurred"
QT_MOC_LITERAL(118, 5) // "error"

    },
    "RtspPlayerWidget\0requestStartStream\0"
    "\0rtspUrl\0requestStopStream\0onPipelineStarted\0"
    "GstElement*\0pipeline\0onErrorOccurred\0"
    "error"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RtspPlayerWidget[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   38,    2, 0x06,    1 /* Public */,
       4,    0,   41,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       5,    1,   42,    2, 0x08,    4 /* Private */,
       8,    1,   45,    2, 0x08,    6 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::QString,    9,

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
        case 2: _t->onPipelineStarted((*reinterpret_cast< std::add_pointer_t<GstElement*>>(_a[1]))); break;
        case 3: _t->onErrorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
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
    }
}

const QMetaObject RtspPlayerWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_RtspPlayerWidget.offsetsAndSize,
    qt_meta_data_RtspPlayerWidget,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_RtspPlayerWidget_t
, QtPrivate::TypeAndForceComplete<RtspPlayerWidget, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<GstElement *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>


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
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 4;
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
