#ifndef __WORKER_HPP__
#define __WORKER_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <QObject>

class QWorker : public QObject
{
    Q_OBJECT

  public:
    /**
     * @brief Construct a new QWorker object
     *
     * @param start_ Start the thread at the object creation
     * @param start_ parent for Qt internal ownership
     */
    QWorker(bool start_, QObject* parent_);
    ~QWorker();

    /**
     * @brief Start the worker thread which waits for tasks
     *
     */
    void start(void);

    /**
     * @brief Finish the thread execution
     *
     */
    void finish(void);

  protected:
    /**
     * @brief Adds the function pointer to the task queue which is processed by
     * the thread (async)
     *
     * @param task_ function pointer to a task
     */
    void addTask(std::function<void()> task_);

  private:
    std::atomic<bool> _alive = std::atomic<bool>(false);
    std::thread _thread;
    std::queue<std::function<void()>> _taskQueue;

    std::mutex _pendingTaskMtx;
    std::queue<std::function<void()>> _pendingTask;
    std::condition_variable _newTaskCv;
    bool _newTask = false;

    void execLoop(void);
};

#endif  // __WORKER_HPP__
