/****************************************************************************
** Meta object code from reading C++ file 'QTreeViewExplorer.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../rover_gui/src/QSshFileExplorer/QTreeViewExplorer.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QTreeViewExplorer.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QTreeViewExplorer_t {
    const uint offsetsAndSize[6];
    char stringdata0[38];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_QTreeViewExplorer_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_QTreeViewExplorer_t qt_meta_stringdata_QTreeViewExplorer = {
    {
QT_MOC_LITERAL(0, 17), // "QTreeViewExplorer"
QT_MOC_LITERAL(18, 18), // "selectionTriggered"
QT_MOC_LITERAL(37, 0) // ""

    },
    "QTreeViewExplorer\0selectionTriggered\0"
    ""
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QTreeViewExplorer[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   20,    2, 0x06,    1 /* Public */,

 // signals: parameters
    QMetaType::Void,

       0        // eod
};

void QTreeViewExplorer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QTreeViewExplorer *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->selectionTriggered(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QTreeViewExplorer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QTreeViewExplorer::selectionTriggered)) {
                *result = 0;
                return;
            }
        }
    }
    (void)_a;
}

const QMetaObject QTreeViewExplorer::staticMetaObject = { {
    QMetaObject::SuperData::link<QTreeView::staticMetaObject>(),
    qt_meta_stringdata_QTreeViewExplorer.offsetsAndSize,
    qt_meta_data_QTreeViewExplorer,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_QTreeViewExplorer_t
, QtPrivate::TypeAndForceComplete<QTreeViewExplorer, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>



>,
    nullptr
} };


const QMetaObject *QTreeViewExplorer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QTreeViewExplorer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QTreeViewExplorer.stringdata0))
        return static_cast<void*>(this);
    return QTreeView::qt_metacast(_clname);
}

int QTreeViewExplorer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTreeView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void QTreeViewExplorer::selectionTriggered()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
