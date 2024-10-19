#include "QWorker.hpp"

#include <rclcpp/rclcpp.hpp>

QWorker::QWorker(bool start_, QObject* parent_): QObject(parent_)
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

void QWorker::addTask(std::function<void()> task_)
{
    if (task_)
    {
        {
            std::lock_guard<std::mutex> lockPendingTask(_pendingTaskMtx);
            _newTask = true;
            _pendingTask.push(task_);
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
                _newTask = false;
                _newTaskCv.wait(lock, [this]() { return _newTask || !_alive; });

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

        std::function<void()> action = _taskQueue.front();
        if (action)
        {
            action();
            _taskQueue.pop();
        }
    }
}
