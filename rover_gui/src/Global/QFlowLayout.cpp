/****
 * QFlowLayout.cpp
 * From the official Qt project examples.
 * Rearranges widgets in a flow layout.
 */

#include "QFlowLayout.hpp"
#include <QtWidgets/QWidget>

QFlowLayout::QFlowLayout(QWidget* parent_, int margin_, int hSpacing_, int vSpacing_):
    QLayout(parent_),
    _m_hSpace(hSpacing_),
    _m_vSpace(vSpacing_)
{
    setContentsMargins(margin_, margin_, margin_, margin_);
}

QFlowLayout::QFlowLayout(int margin_, int hSpacing_, int vSpacing_):
    _m_hSpace(hSpacing_),
    _m_vSpace(vSpacing_)
{
    setContentsMargins(margin_, margin_, margin_, margin_);
}

QFlowLayout::~QFlowLayout()
{
    QLayoutItem* item;
    while ((item = takeAt(0)))
        delete item;
}

void QFlowLayout::addItem(QLayoutItem* item_)
{
    _itemList.append(item_);
}

int QFlowLayout::horizontalSpacing() const
{
    if (_m_hSpace >= 0)
    {
        return _m_hSpace;
    }
    else
    {
        return smartSpacing(QStyle::PM_LayoutHorizontalSpacing);
    }
}

int QFlowLayout::verticalSpacing() const
{
    if (_m_vSpace >= 0)
    {
        return _m_vSpace;
    }
    else
    {
        return smartSpacing(QStyle::PM_LayoutVerticalSpacing);
    }
}

int QFlowLayout::count() const
{
    return _itemList.size();
}

QLayoutItem* QFlowLayout::itemAt(int index_) const
{
    return _itemList.value(index_);
}

QLayoutItem* QFlowLayout::takeAt(int index_)
{
    if (index_ >= 0 && index_ < _itemList.size())
        return _itemList.takeAt(index_);
    return nullptr;
}

Qt::Orientations QFlowLayout::expandingDirections() const
{
    return {};
}

bool QFlowLayout::hasHeightForWidth() const
{
    return true;
}

int QFlowLayout::heightForWidth(int width_) const
{
    int height = doLayout(QRect(0, 0, width_, 0), true);
    return height;
}

void QFlowLayout::setGeometry(const QRect& rect_)
{
    QLayout::setGeometry(rect_);
    doLayout(rect_, false);
}

QSize QFlowLayout::sizeHint() const
{
    return minimumSize();
}

QSize QFlowLayout::minimumSize() const
{
    QSize size;
    for (const QLayoutItem* item : std::as_const(_itemList))
        size = size.expandedTo(item->minimumSize());

    const QMargins margins = contentsMargins();
    size += QSize(margins.left() + margins.right(), margins.top() + margins.bottom());
    return size;
}

int QFlowLayout::doLayout(const QRect& rect_, bool testOnly_) const
{
    int left, top, right, bottom;
    getContentsMargins(&left, &top, &right, &bottom);
    QRect effectiveRect = rect_.adjusted(+left, +top, -right, -bottom);
    int x = effectiveRect.x();
    int y = effectiveRect.y();
    int lineHeight = 0;

    for (QLayoutItem* item : std::as_const(_itemList))
    {
        const QWidget* wid = item->widget();
        int spaceX = horizontalSpacing();
        if (spaceX == -1)
            spaceX = wid->style()->layoutSpacing(QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Horizontal);
        int spaceY = verticalSpacing();
        if (spaceY == -1)
            spaceY = wid->style()->layoutSpacing(QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Vertical);
        int nextX = x + item->sizeHint().width() + spaceX;
        if (nextX - spaceX > effectiveRect.right() && lineHeight > 0)
        {
            x = effectiveRect.x();
            y = y + lineHeight + spaceY;
            nextX = x + item->sizeHint().width() + spaceX;
            lineHeight = 0;
        }

        if (!testOnly_)
            item->setGeometry(QRect(QPoint(x, y), item->sizeHint()));

        x = nextX;
        lineHeight = qMax(lineHeight, item->sizeHint().height());
    }
    return y + lineHeight - rect_.y() + bottom;
}

int QFlowLayout::smartSpacing(QStyle::PixelMetric pm_) const
{
    QObject* parent = this->parent();
    if (!parent)
    {
        return -1;
    }
    else if (parent->isWidgetType())
    {
        QWidget* pw = static_cast<QWidget*>(parent);
        return pw->style()->pixelMetric(pm_, nullptr, pw);
    }
    else
    {
        return static_cast<QLayout*>(parent)->spacing();
    }
}