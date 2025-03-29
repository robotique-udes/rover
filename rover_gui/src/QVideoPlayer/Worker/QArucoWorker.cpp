#include "QArucoWorker.hpp"

#include <rclcpp/rclcpp.hpp>

QWorker::QWorker(bool start_, QObject* parent_):
    QObject(parent_)
{
    if (start_)
    {
        this->start();
    }
}

QWorker::~QWorker()
{
    this->finish();
}

void QWorker::start(void)
{
    if (!_thread.joinable())
    {
        _alive = true;
        _thread = std::thread(&QWorker::execLoop, this);
    }
}

void QWorker::finish(void)
{
    _alive = false;
    _newTaskCv.notify_one();
    if (_thread.joinable())
    {
        _thread.join();
    }
}

void QWorker::cancelCurrentTasks()
{
    _cancelCurrentTasksFlag.store(true);
}

void QWorker::cancelAllTasks()
{
    _cancelAllTasksFlag.store(true);
    this->cancelCurrentTasks();
}

size_t QWorker::getTaskNb(void)
{
    return _taskQueueSize.load();
}

void QWorker::addTask(std::function<void()> task_)
{
    if (task_)
    {
        {
            std::lock_guard<std::mutex> lockPendingTask(_pendingTaskMtx);
            _newTaskFlag = true;
            _pendingTask.push(task_);
            _taskQueueSize.store(_taskQueueSize.load() + 1);
        }
        _newTaskCv.notify_one();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "QWorker::addTask(): Nullptr is not a valid task, nothing done");
    }
}

void QWorker::execLoop(void)
{
    while (_alive)
    {
        {
            std::unique_lock<std::mutex> lock(_pendingTaskMtx);
            if (_pendingTask.empty() && _taskQueue.empty())
            {
                _taskQueueSize.store(_taskQueue.size());
                emit this->allTasksDone();

                _newTaskFlag = false;
                _newTaskCv.wait(lock,
                                [this]()
                                {
                                    return _newTaskFlag || !_alive;
                                });
                _cancelAllTasksFlag.store(false);
                _cancelCurrentTasksFlag.store(false);

                if (!_alive)
                {
                    return;
                }
            }

            while (!_pendingTask.empty())
            {
                _taskQueue.push(_pendingTask.front());
                _pendingTask.pop();
            }
        }

        _taskQueueSize.store(_taskQueue.size());

        std::function<void()> action = _taskQueue.front();
        if (action && !_cancelAllTasksFlag.load())
        {
            action();
            _cancelCurrentTasksFlag.store(false);
            _taskQueue.pop();
        }
        else if (action)
        {
            _taskQueue.pop();
        }
    }
}
