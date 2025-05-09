#ifndef SHARED_MSG_HPP
#define SHARED_MSG_HPP

#include <rover_lib2/helpers/assert.hpp>
#include <rover_lib2/helpers/macros.hpp>
#include <rover_lib2/helpers/thread_safe_access.hpp>

#include <rclcpp/rclcpp.hpp>

#include <list>
#include <mutex>

namespace CanMaster
{
    template<typename MsgT>
    class SharedRosMsg
    {
      public:
        /**
         * @brief Attach a new publisher on this message, only if its not already attached (the fastest rate will be applied to
         * all passed publishers)
         *
         * @param pNode_ Pointer to a valid rclcpp::node, must be valid the first time the method is called and always be the same
         * node afterwards (or nullptr)
         * @param wpPub_ Weak pointer to a valid rclcpp::publisher, if multiple publisher on same topic only one will be kept.
         * @param rate_ Publishing rate at wish all publishers will publish. If multiple rate are passed only the fastest will be
         * applied
         */
        bool attachNewPub(std::shared_ptr<rclcpp::Node> pNode_, std::weak_ptr<rclcpp::Publisher<MsgT>> wpPub_, float rate_)
        {
            ASSERT_COND_MSG((!_node && pNode_) || (pNode_ && _node && pNode_ == _node),
                            "A valid node must be passed and it must always be the same one");
            _node = pNode_;

            if (rate_ < 0.0F || rate_ > 1000.0F)
            {
                RCLCPP_WARN(_node->get_logger(), "Expected publish rate be in range [0, 1000]. Value asked by user: %.2f", rate_);
            }

            rate_ = CONSTRAIN(rate_, 0.0F, 1000.0F);
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

            this->removeNullPub();
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

        ThreadSafeAccess<MsgT> get()
        {
            return ThreadSafeAccess<MsgT>(_msgMutex, _msg);
        }

      private:
        void publish(void)
        {
            ThreadSafeAccess<MsgT> msgLocked = this->get();
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

        /**
         * @brief Parse the list of publisher and removes them if they're not valid anymore
         *
         */
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
