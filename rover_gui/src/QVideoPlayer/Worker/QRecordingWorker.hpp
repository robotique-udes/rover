#ifndef RECORDING_WORKER_HPP
#define RECORDING_WORKER_HPP

#include <QObject>
#include <QString>

#include "Global/Workers/QWorker.hpp"

class QRecordingWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint64_t MAX_DELAY_SERVICE_CALL = 4'000UL;
    static constexpr uint16_t SERVICE_POLL_INTERVAL = 100U;

  public:
    QRecordingWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QRecordingWorker();

};
#endif // RECORDING_WORKER_HPP