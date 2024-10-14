#ifndef __WORKER_HPP__
#define __WORKER_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include <QObject>

/**
 * @brief Worker class which handles an async task queue.
 *
 * Designed to be used as a parent class for other classes that need
 * asynchronous task handling.
 *
 * @example
 *     #include "worker.hpp"
 *     #include <iostream>
 *     
 *     class MyWorker : public Worker
 *     {
 *       public:
 *         MyWorker(): Worker(true) {}
 *     
 *         void addPrintTask(const std::string& message)
 *         {
 *             addTask([message]() { std::cout << "Task says: " << message << std::endl; });
 *         }
 *     };
 *     
 *     int main()
 *     {
 *         MyWorker worker;
 *     
 *         // Add some tasks
 *         worker.addPrintTask("Hello from task 1");
 *         worker.addPrintTask("Hello from task 2");
 *     
 *         // Give time for tasks to be processed
 *         std::this_thread::sleep_for(std::chrono::seconds(1));
 *     
 *         // Finish the worker
 *         worker.finish();
 *     
 *         return 0;
 *     }
 */

class Worker : public QObject
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
