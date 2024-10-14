#ifndef __WORKER_HPP__
#define __WORKER_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <QObject>

/**
 * @brief Worker class which handles a async task queue
 *
 */
class Worker: public QObject
{
    Q_OBJECT

  public:
    /**
     * @brief Construct a new Worker object
     *
     * @param start_ Optional start the thread at the object creation
     */
    Worker(bool start_ = false, QObject* parent_ = nullptr);
    ~Worker();

    /**
     * @brief Start the worker thread which waits for tasks
     *
     */
    void start(void);

    /**
     * @brief Finish the thread execution
     *
     * @param join_ Optional join of the thread at user discretion
     */
    void finish();

  protected:
    /**
     * @brief Adds the function pointer to the task queue which is processed by
     * the thread (async)
     *
     * @param task_ function pointer to a task
     */
    void addTask(std::function<void()> task_);

  private:
    void execLoop(void);

    std::atomic<bool> _alive = std::atomic<bool>(false);
    std::thread _thread;
    std::queue<std::function<void()>> _taskQueue;

    std::mutex _pendingTaskMtx;
    std::queue<std::function<void()>> _pendingTask;
    std::condition_variable _newTaskCv;
    bool _newTask = false;
};

#endif  // __WORKER_HPP__
