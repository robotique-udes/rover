#ifndef GLOBAL_QFLOWLAYOUT_HPP
#define GLOBAL_QFLOWLAYOUT_HPP

#include <QtWidgets/QLayout>
#include <QtWidgets/QStyle>

class QFlowLayout : public QLayout
{
  public:
    explicit QFlowLayout(QWidget* parent_, int margin_ = -1, int hSpacing_ = -1, int vSpacing_ = -1);
    explicit QFlowLayout(int margin_ = -1, int hSpacing_ = -1, int vSpacing_ = -1);
    ~QFlowLayout();

    void addItem(QLayoutItem* item_) override;
    int horizontalSpacing() const;
    int verticalSpacing() const;
    Qt::Orientations expandingDirections() const override;
    bool hasHeightForWidth() const override;
    int heightForWidth(int) const override;
    int count() const override;
    QLayoutItem* itemAt(int index_) const override;
    QSize minimumSize() const override;
    void setGeometry(const QRect& rect) override;
    QSize sizeHint() const override;
    QLayoutItem* takeAt(int index_) override;

  private:
    int doLayout(const QRect& rect_, bool testOnly_) const;
    int smartSpacing(QStyle::PixelMetric pm_) const;

    QList<QLayoutItem*> _itemList;
    int _m_hSpace;
    int _m_vSpace;
};

#endif  // GLOBAL_QFLOWLAYOUT_HPP