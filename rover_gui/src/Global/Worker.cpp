#include "Worker.hpp"

Worker::Worker(bool start_, QObject* parent_): QObject(parent_)
{
    if (start_)
    {
        this->start();
    }
}

Worker::~Worker()
{
    this->finish();
}

void Worker::start(void)
{
    if (!_thread.joinable())
    {
        _alive = true;
        _thread = std::thread(&Worker::execLoop, this);
    }
}

void Worker::finish()
{
    _alive = false;
    if (_thread.joinable())
    {
        _thread.join();
    }
}

void Worker::addTask(std::function<void()> task_)
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
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Worker::addTask(): Nullptr is not a valid task, nothing done");
    }
}

void Worker::execLoop(void)
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
