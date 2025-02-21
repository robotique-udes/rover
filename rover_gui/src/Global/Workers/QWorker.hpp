#ifndef __WORKER_HPP__
#define __WORKER_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <QObject>
#include <QProgressBar>
#include <QString>

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

    /**
     * @brief [THREAD_SAFE] Cancel current running task, it's the users job to add exit
     * conditions to their tasks when _cancelCurrentTasksFlag flag is false
     *
     */
    void cancelCurrentTasks(void);

    /**
     * @brief [THREAD_SAFE] Cancel current running task and followings, it's the users job to
     * add exit conditions to their tasks when _cancelCurrentTasksFlag flag is
     * false
     *
     */
    void cancelAllTasks(void);

    /**
     * @brief [THREAD_SAFE] Return current estimate of the number of task
     * running and pending.
     *
     */
    size_t getTaskNb(void);

  signals:
    void allTasksDone(void);

  protected:
    /**
     * @brief [THREAD_SAFE] Adds the function pointer to the task queue which is processed by
     * the thread (async)
     *
     * @param task_ function pointer to a task
     */
    void addTask(std::function<void()> task_);

    std::atomic<bool> _cancelCurrentTasksFlag = false;

  private:
    void execLoop(void);

    std::atomic<bool> _alive = std::atomic<bool>(false);
    std::thread _thread;
    std::queue<std::function<void()>> _taskQueue;
    std::atomic<size_t> _taskQueueSize = 0u;

    std::mutex _pendingTaskMtx;
    std::queue<std::function<void()>> _pendingTask;
    std::condition_variable _newTaskCv;
    bool _newTaskFlag = false;
    std::atomic<bool> _cancelAllTasksFlag = false;
};

#endif  // __WORKER_HPP__
