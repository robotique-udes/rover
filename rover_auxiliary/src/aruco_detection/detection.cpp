#include "detection.hpp"

ArucoDetection::ArucoDetection(std::string _cameraURL): processedFrame(_cameraURL) {}
ArucoDetection::~ArucoDetection() {}

std::vector<uint16_t> ArucoDetection::detect(bool DEBUG_MODE)
{
    processedFrame.processFrame(DEBUG_MODE);
    if (!processedFrame.IdsEmpty())
    {  // If ids were detected
        if (DEBUG_MODE)
        {
            cv::Mat frame = processedFrame.processFrame(DEBUG_MODE).value_or(cv::Mat());
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(30);
        }

        return processedFrame.getIds();
    }

    return {};  // Return empty vector if no ids detected
}

/*
Look for marker in a new frame every 100 ms
Id has to be detected 10 times to be validated (not necessarily consective)
If Id is not found in the next frame, it has 10 frame to be found again (strike), else the count is reset
*/
std::vector<uint16_t> ArucoDetection::update(bool DEBUG_MODE)
{
    std::vector<uint16_t> validatedIds;
    std::vector<uint16_t> detectedIds = detect(DEBUG_MODE);

    for (auto it = validation.begin(); it != validation.end();)
    {
        uint16_t id = it->first;

        auto found = std::find(detectedIds.begin(), detectedIds.end(), id);

        if (found != detectedIds.end())
        {
            it->second.addValue(static_cast<uint16_t>(1));
            detectedIds.erase(found);

            if (it->second.getAverage() > VALIDATION_THRESHOLD)
            {
                validatedIds.push_back(id);
            }

            ++it;
        }
        else
        {
            it->second.addValue(static_cast<uint16_t>(0));

            if (it->second.getAverage() == 0)
            {
                it = validation.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    for (const auto& id : detectedIds)
    {
        validation.emplace(id, MovingAverage<uint16_t, COEFF_NB_ARUCO>(0));
    }

    RoverLib::Timer<uint64_t, RoverLib::millis> timer(DELAY_BETWEEN_CAPTURE_MS);
    
    while (!timer.isDone())
    {
    }

    return validatedIds;
}
