#ifndef SHARED_MSG_HPP
#define SHARED_MSG_HPP

#include <rover_lib2/helpers/assert.hpp>
#include <rover_lib2/helpers/macros.hpp>

#include <rclcpp/rclcpp.hpp>

#include <list>
#include <mutex>

namespace CanMaster
{
    template<typename T>
    class LockedAccess
    {
      public:
        LockedAccess(std::mutex& mtx_, T& data_):
            _lock(mtx_),
            _data(data_)
        {
        }

        T& getThreadSafeAccess(void)
        {
            return _data;
        }

      private:
        std::lock_guard<std::mutex> _lock;
        T& _data;
    };

    template<typename MsgT>
    class SharedRosMsg
    {
      public:
        /**
         * @brief Attach a new publisher on this message, only if its not already attached
         *
         * @param pub_

         */
        bool attachNewPub(std::shared_ptr<rclcpp::Node> pNode_, std::weak_ptr<rclcpp::Publisher<MsgT>> wpPub_, float rate_)
        {
            rate_ = CONSTRAIN(rate_, 0.0F, 1000.0F);

            ASSERT_COND_MSG((!_node && pNode_) || (pNode_ && _node && pNode_ == _node),
                            "A valid node must be passed and it must always be the same one");
            _node = pNode_;

            if (_node || rate_ > _rate)
            {
                _rate = rate_;

                if (_timer)
                {
                    _timer->reset();
                }

                uint64_t periodMs = static_cast<uint64_t>(1'000.0F / rate_);
                _timer = _node->create_wall_timer(std::chrono::milliseconds(periodMs),
                                                  [this](void)
                                                  {
                                                      this->publish();
                                                  });
            }

            this->removeNullPub();
            std::string newPubTopicName = "";
            if (wpPub_.expired())
            {
                return false;
            }
            else
            {
                auto spPub = wpPub_.lock();
                newPubTopicName = spPub->get_topic_name();
            }

            const auto& it = std::find_if(_pubList.begin(),
                                          _pubList.end(),
                                          [&](std::weak_ptr<rclcpp::Publisher<MsgT>> wpPub)
                                          {
                                              if (wpPub.expired())
                                              {
                                                  return false;
                                              }
                                              else
                                              {
                                                  auto pPub_ = wpPub.lock();
                                                  if (!pPub_)
                                                  {
                                                      return false;
                                                  }
                                                  else
                                                  {
                                                      return (pPub_->get_topic_name() == newPubTopicName);
                                                  }
                                              }
                                          });
            if (it == _pubList.end())
            {
                _pubList.push_back(wpPub_);
                return true;
            }
            else
            {
                return false;
            }
        }

        void removePub(std::weak_ptr<rclcpp::Publisher<MsgT>> pub_)
        {
            auto it = std::find_if(_pubList.begin(),
                                   _pubList.end(),
                                   [&](const std::weak_ptr<rclcpp::Publisher<MsgT>>& wp)
                                   {
                                       if (wp.expired())
                                       {
                                           return false;
                                       }
                                       auto sp = wp.lock();
                                       return (sp == pub_.lock());
                                   });
            if (it != _pubList.end())
            {
                _pubList.erase(it);
            }
        }

        LockedAccess<MsgT> get()
        {
            return LockedAccess<MsgT>(_msgMutex, _msg);
        }

      private:
        void publish(void)
        {
            LockedAccess<MsgT> msgLocked = get();
            MsgT msg = msgLocked.getThreadSafeAccess();
            for (auto& wpPub : _pubList)
            {
                if (wpPub.expired())
                {
                    this->removeNullPub();
                }

                auto spSub = wpPub.lock();
                if (spSub)
                {
                    spSub->publish(msg);
                }
            }
        }

        void removeNullPub(void)
        {
            _pubList.erase(std::remove_if(_pubList.begin(),
                                          _pubList.end(),
                                          [](const std::weak_ptr<rclcpp::Publisher<MsgT>>& wptr)
                                          {
                                              return wptr.expired();
                                          }),
                           _pubList.end());
        }

        MsgT _msg;
        std::shared_ptr<rclcpp::Node> _node;
        rclcpp::TimerBase::SharedPtr _timer;

        bool _alive;
        std::thread _publishThread;
        std::condition_variable _cvPublishThread;
        std::mutex _msgMutex;

        std::list<std::weak_ptr<rclcpp::Publisher<MsgT>>> _pubList;

        float _rate;
    };
}  // namespace CanMaster

#endif  // SHARED_MSG_HPP
