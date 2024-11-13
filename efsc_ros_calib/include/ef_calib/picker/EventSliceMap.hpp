#ifndef ef_calib_EventSliceMap_HPP
#define ef_calib_EventSliceMap_HPP

#include <shared_mutex>
#include <map>
#include <iterator>
//#include <set>
#include <memory>

#include <ef_calib/picker/EventSlice.hpp>

namespace ef_calib {
    namespace picker {
    class EventSliceMap {
    public:
        typedef std::shared_ptr<EventSliceMap> Ptr;

        EventSliceMap():subMap(nullptr){};

        // EventSliceMutex_
        inline void EventSliceLockShared() const {
            EventSliceMutex_.lock_shared();
        }
        inline void EventSliceUnlockShared() const {
            EventSliceMutex_.unlock_shared();
        }

        //---------------------------- single EventSlice --------------------------------//
        inline void addEventSlice(const EventSlice::Ptr &es){
            std::unique_lock lock(EventSliceMutex_);
            es_vector[es->time] = es;
        }
        inline bool hasEventSlice(uint64_t timestamp) const {
            std::shared_lock lock(EventSliceMutex_);
            return es_vector.find(timestamp) != es_vector.end();
        }
        inline EventSlice::Ptr getEventSlice(uint64_t timestamp) const {
            std::shared_lock lock(EventSliceMutex_);
            auto result = es_vector.find(timestamp);
            return result == es_vector.end() ? nullptr : result->second;
        }
        inline EventSlice::Ptr firstEventSlice() const {
            std::shared_lock lock(EventSliceMutex_);
            if (es_vector.empty())
                return nullptr;
            else
                return es_vector.cbegin()->second;
        }
        inline EventSlice::Ptr lastEventSlice() const {
            std::shared_lock lock(EventSliceMutex_);
            if (es_vector.empty())
                return nullptr;
            else
                return es_vector.crbegin()->second;
        }
        inline bool empty() const {
            std::shared_lock lock(EventSliceMutex_);
            return es_vector.empty();
        }
        void removeEventSlice(uint64_t timestamp){
            // erase the keyframes_(timestamp) until there is no keyframes_(timestamp) in it
            EventSliceMap *map = this;
            while (map != nullptr) {
                if (map->hasEventSlice(timestamp)) {
                    map->EventSliceMutex_.lock();
                    map->es_vector.erase(timestamp);
                    map->EventSliceMutex_.unlock();

                    map = map->subMap.get();
                } else {
                    break;
                }
            }
        }

        //----------------------------- multiple EventSlices --------------------------------//
        //require shared mutex when using.
        inline const std::map<uint64_t, EventSlice::Ptr> &EventSlices() const {
            return es_vector;
        }
        inline std::map<uint64_t, EventSlice::Ptr> copyEventSlices() const {
            std::shared_lock lock(EventSliceMutex_);
            return es_vector;
        }
        inline int SliceNum() const {
            std::shared_lock lock(EventSliceMutex_);
            return es_vector.size();
        }


        inline std::map<uint64_t, EventSlice::Ptr> EventSlicebefore(uint64_t timestamp, uint64_t remove_time){
            std::shared_lock lock(EventSliceMutex_);
            auto itr =  es_vector.lower_bound(timestamp);
            if(itr != es_vector.end()){
                std::map<uint64_t, EventSlice::Ptr> es_buffer;
                for (auto Erased_itr = es_vector.cbegin(); Erased_itr->first < timestamp; ++Erased_itr) {
                    if(Erased_itr->first <= remove_time)
                        requestRemoveEventSlice(Erased_itr->first);
                    es_buffer[Erased_itr->first] = Erased_itr->second;
                    // std::cout<< (timestamp - Erased_itr->first)/1000<<"ms ";
                }
                // std::cout<<" not end"<<std::endl;
                return es_buffer;
            }
            else {
                if(es_vector.empty())
                    return es_vector;
                for (auto Erased_itr = es_vector.cbegin(); (Erased_itr!= es_vector.cend()) && (Erased_itr->first <= remove_time); ++Erased_itr){
                    // std::cout<< (timestamp - Erased_itr->first)/1000<<"ms ";
                    requestRemoveEventSlice(Erased_itr->first);
                }
                // std::cout<<" end"<<std::endl;
                return es_vector;
            }      
        }

        inline std::pair<int64_t, int64_t> EventSliceWindow(uint64_t startTime, uint64_t endTime) {
            std::shared_lock lock(EventSliceMutex_);
            int64_t startIdx = 0;
            int64_t endIdx = 0;
            auto itr =  es_vector.lower_bound(endTime);
            if (itr != es_vector.end()){
                for (auto Erased_itr = es_vector.cbegin(); Erased_itr->first <= endTime; ++Erased_itr) {
                    requestRemoveEventSlice(Erased_itr->first);
                    if(Erased_itr->first > startTime && startIdx == 0) startIdx = Erased_itr->first;
                    endIdx = Erased_itr->first;
                }
                if( endIdx <= startTime)
                    return std::make_pair(0, 0);
                else
                    return std::make_pair(startIdx, endIdx);
            }
            else {// all the elements in esMap <=timestamp
                if (es_vector.empty())
                    return std::make_pair(0, 0);
                
                for (auto Erased_itr = es_vector.cbegin(); Erased_itr!= es_vector.cend(); ++Erased_itr) {
                    requestRemoveEventSlice(Erased_itr->first);
                    if(Erased_itr->first > startTime && startIdx == 0) startIdx = Erased_itr->first;
                    endIdx = Erased_itr->first;
                }
                if( endIdx <= startTime)
                    return std::make_pair(0, 0);
                else
                    return std::make_pair(startIdx, endIdx);
            }
        }
        
        //*******@brief clear map
        void clear() {
            EventSliceMap *map = this;
            while (map != nullptr) {
                std::unique_lock lock(map->EventSliceMutex_);
                map->es_vector.clear();
                map = map->subMap.get();
            }
        }

        //*******@brief Erase EventSlices in RemoveList.
        inline void cleanMap() {
            EventSlicesToBeErasedMutex_.lock();
            for (uint64_t timeStamp: EventSlicesToBeErased_) {
                removeEventSlice(timeStamp);
            }
            EventSlicesToBeErased_.clear();
            EventSlicesToBeErasedMutex_.unlock();
        }
        //*******@brief Add keyframe to RemoveList. Remember to clear observations.
        inline void requestRemoveEventSlice(uint64_t timestamp) {
            if (auto bf = getEventSlice(timestamp)) {
                std::scoped_lock lock(EventSlicesToBeErasedMutex_);
                EventSlicesToBeErased_.push_back(timestamp);
            }
        }
/*
        static void
        switchOptData(const std::map<double, ef_Frames::Ptr> &keyframes,
                      const std::map<int, LandmarkBase::Ptr> &landmarks);
*/

        EventSliceMap::Ptr subMap; // parent have no mutex control over child, but part elements copied to child

    protected:
        std::map<uint64_t, EventSlice::Ptr> es_vector;
        mutable std::shared_mutex EventSliceMutex_;

        std::vector<uint64_t> EventSlicesToBeErased_;
        mutable std::mutex EventSlicesToBeErasedMutex_;
    };
} // picker namespace
}

#endif //ef_calib_EventSliceMap_HPP
